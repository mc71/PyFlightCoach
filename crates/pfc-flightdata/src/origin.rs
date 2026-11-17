use pfc_geometry::{euler_angles, Point, Quaternion, GPS};
use pfc_schemas::fcj::{Origin as FcjOrigin, Parameters as FcjParameters};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;
use std::path::Path;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct GpsDict {
    lat: f64,
    #[serde(alias = "lng")]
    long: f64,
    alt: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct OriginDict {
    name: String,
    pos: GpsDict,
    heading: f64,
}

/// Aerobatic box reference frame defined by pilot GPS position and viewing heading (radians CW from North).
#[derive(Debug, Clone, PartialEq)]
pub struct Origin {
    pub name: String,
    pub pos: GPS,
    /// Direction the pilot is facing (in radians clockwise from North).
    pub heading: f64,
}

impl Serialize for Origin {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let od = OriginDict {
            name: self.name.clone(),
            pos: GpsDict {
                lat: self.lat(),
                long: self.long(),
                alt: self.alt(),
            },
            heading: self.heading,
        };
        od.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Origin {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let od = OriginDict::deserialize(deserializer)?;
        Ok(Self {
            name: od.name,
            pos: GPS::new(od.pos.lat, od.pos.long, od.pos.alt),
            heading: od.heading,
        })
    }
}

impl Origin {
    pub fn new(name: impl Into<String>, pos: GPS, heading: f64) -> Self {
        Self {
            name: name.into(),
            pos,
            heading,
        }
    }

    pub fn zero() -> Self {
        Self::new("zero", GPS::new(0.0, 0.0, 0.0), 0.0)
    }

    #[inline]
    pub fn lat(&self) -> f64 {
        self.pos.lat0()
    }

    #[inline]
    pub fn long(&self) -> f64 {
        self.pos.long0()
    }

    #[inline]
    pub fn alt(&self) -> f64 {
        self.pos.alt0()
    }

    #[inline]
    pub fn pilot_position(&self) -> &GPS {
        &self.pos
    }

    /// Rotation quaternion from NED frame to Box frame (`+X` Right, `+Y` Out, `+Z` Up).
    pub fn rotation(&self) -> Quaternion {
        euler_angles(PI, 0.0, self.heading + PI / 2.0)
    }

    pub fn gps_to_point(&self, gps: &GPS) -> Point {
        self.rotation().transform_point(&(gps - &self.pos))
    }

    pub fn from_points(name: impl Into<String>, pilot: &GPS, centre: &GPS) -> Self {
        let direction = centre - pilot;
        let heading = direction.y0().atan2(direction.x0());
        Self::new(name, pilot.row(0), heading)
    }

    pub fn from_f3a_zone_str(content: &str) -> Result<Self, String> {
        let lines: Vec<&str> = content.lines().map(|l| l.trim()).collect();
        if lines.len() < 7 {
            return Err("Invalid .f3a Zone Pro file: expected at least 7 lines".to_string());
        }
        let name = lines[1].to_string();
        let pilot_lat: f64 = lines[2].parse().map_err(|e| format!("pilot_lat: {}", e))?;
        let pilot_lon: f64 = lines[3].parse().map_err(|e| format!("pilot_lon: {}", e))?;
        let centre_lat: f64 = lines[4].parse().map_err(|e| format!("centre_lat: {}", e))?;
        let centre_lon: f64 = lines[5].parse().map_err(|e| format!("centre_lon: {}", e))?;
        let alt: f64 = lines[6].parse().map_err(|e| format!("alt: {}", e))?;
        Ok(Self::from_points(
            name,
            &GPS::new(pilot_lat, pilot_lon, alt),
            &GPS::new(centre_lat, centre_lon, alt),
        ))
    }

    pub fn from_f3a_zone(path: impl AsRef<Path>) -> Result<Self, String> {
        let content = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        Self::from_f3a_zone_str(&content)
    }

    pub fn to_f3a_zone_string(&self) -> String {
        let centre = self.pos.offset(&Point::new(
            100.0 * self.heading.cos(),
            100.0 * self.heading.sin(),
            0.0,
        ));
        format!(
            "Emailed box data for F3A Zone Pro - please DON'T modify!\n{}\n{}\n{}\n{}\n{}\n{}\n",
            self.name,
            self.lat(),
            self.long(),
            centre.lat0(),
            centre.long0(),
            self.alt()
        )
    }

    pub fn to_f3a_zone(&self, path: impl AsRef<Path>) -> Result<(), String> {
        std::fs::write(path, self.to_f3a_zone_string()).map_err(|e| e.to_string())
    }

    pub fn from_fcjson_parameters(parms: &FcjParameters) -> Self {
        Self::new(
            "FCJ_box",
            GPS::new(parms.pilotLat, parms.pilotLng, parms.pilotAlt),
            parms.rotation,
        )
    }

    pub fn from_fcj_origin(fcj_origin: &FcjOrigin) -> Self {
        let pos = GPS::new(fcj_origin.lat, fcj_origin.lng, fcj_origin.alt).offset(&Point::new(
            fcj_origin.move_north,
            fcj_origin.move_east,
            0.0,
        ));
        Self::new("FCJ_Origin", pos, fcj_origin.heading.to_radians())
    }

    pub fn to_fcj_origin(&self) -> FcjOrigin {
        FcjOrigin {
            lat: self.lat(),
            lng: self.long(),
            alt: self.alt(),
            heading: self.heading.to_degrees(),
            move_north: 0.0,
            move_east: 0.0,
            move_down: 0.0,
        }
    }
}
