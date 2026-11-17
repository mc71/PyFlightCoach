use crate::point::Point;
use serde::{Deserialize, Serialize};
use std::ops::{Index, Sub};

pub const ERAD: f64 = 6_378_100.0;
pub const LOCFAC: f64 = ERAD * (std::f64::consts::PI / 180.0);

#[inline]
pub fn safecos(angle_deg: f64) -> f64 {
    angle_deg.to_radians().cos().max(0.01)
}

/// Geodetic coordinates `[(lat_deg, long_deg, alt_m_msl); N]` with ArduPilot-compatible NED projection.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GPS {
    pub data: Vec<[f64; 3]>,
}

impl GPS {
    pub fn new(lat: f64, long: f64, alt: f64) -> Self {
        Self {
            data: vec![[lat, long, alt]],
        }
    }

    pub fn from_rows(data: Vec<[f64; 3]>) -> Self {
        Self { data }
    }

    pub fn from_columns(lat: &[f64], long: &[f64], alt: &[f64]) -> Self {
        let n = lat.len().max(long.len()).max(alt.len());
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            data.push([
                lat[i.min(lat.len() - 1)],
                long[i.min(long.len() - 1)],
                alt[i.min(alt.len() - 1)],
            ]);
        }
        Self { data }
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.data.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    pub fn row(&self, idx: usize) -> Self {
        Self {
            data: vec![self.data[idx]],
        }
    }

    pub fn lat(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[0]).collect()
    }

    pub fn long(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[1]).collect()
    }

    pub fn alt(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[2]).collect()
    }

    #[inline]
    pub fn lat0(&self) -> f64 {
        self.data[0][0]
    }

    #[inline]
    pub fn long0(&self) -> f64 {
        self.data[0][1]
    }

    #[inline]
    pub fn alt0(&self) -> f64 {
        self.data[0][2]
    }

    /// Offset GPS coordinates by a local NED `Point` `(North, East, Down)`.
    pub fn offset(&self, pin: &Point) -> Self {
        let n = self.len().max(pin.len());
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            let [lat, lon, alt] = self.data[i.min(self.len() - 1)];
            let [px, py, pz] = pin.data[i.min(pin.len() - 1)];
            let latb = lat + px / LOCFAC;
            let lonb = lon + py / (LOCFAC * safecos(latb));
            let altb = alt - pz;
            data.push([latb, lonb, altb]);
        }
        Self { data }
    }

    pub fn bspline(&self, index: &[f64], target_ts: &[f64]) -> Self {
        if self.is_empty() {
            return self.clone();
        }
        let g0 = self.row(0);
        let ned = self - &g0;
        let interp_ned = ned.bspline(index, target_ts);
        g0.offset(&interp_ned)
    }
}

impl Index<usize> for GPS {
    type Output = [f64; 3];
    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}

impl Sub<&GPS> for &GPS {
    type Output = Point;
    fn sub(self, rhs: &GPS) -> Point {
        let n = self.len().max(rhs.len());
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            let [lat_a, lon_a, alt_a] = self.data[i.min(self.len() - 1)];
            let [lat_b, lon_b, alt_b] = rhs.data[i.min(rhs.len() - 1)];
            let dx = (lat_a - lat_b) * LOCFAC;
            let dy = (lon_a - lon_b) * LOCFAC * safecos(lat_a);
            let dz = alt_b - alt_a;
            data.push([dx, dy, dz]);
        }
        Point::from_rows(data)
    }
}

impl Sub<GPS> for GPS {
    type Output = Point;
    fn sub(self, rhs: GPS) -> Point {
        (&self) - (&rhs)
    }
}
