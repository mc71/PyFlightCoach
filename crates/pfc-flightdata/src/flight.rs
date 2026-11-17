use crate::origin::Origin;
use ardupilot_log_reader::Ardupilot;
use indexmap::IndexMap;
use pfc_geometry::{butterworth_lowpass_filtfilt, unwrap_1d, Point, GPS};
use pfc_schemas::fcj::FCJ;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::f64::consts::PI;
use std::path::Path;

pub fn flightmode_name(mode_num: i64) -> &'static str {
    match mode_num {
        0 => "MANUAL",
        1 => "CIRCLE",
        2 => "STABILIZE",
        3 => "TRAINING",
        4 => "ACRO",
        5 => "FBWA",
        6 => "FBWB",
        7 => "CRUISE",
        8 => "AUTOTUNE",
        10 => "AUTO",
        11 => "RTL",
        12 => "LOITER",
        13 => "TAKEOFF",
        14 => "AVOID_ADSB",
        15 => "GUIDED",
        16 => "INITIALISING",
        17 => "QSTABILIZE",
        18 => "QHOVER",
        19 => "QLOITER",
        20 => "QLAND",
        21 => "QRTL",
        22 => "QAUTOTUNE",
        23 => "QACRO",
        24 => "THERMAL",
        _ => "UNKNOWN",
    }
}

/// Time-aligned flight telemetry extracted from an ArduPilot `.BIN` log or FlightCoach JSON (`FCJ`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Flight {
    pub data: IndexMap<String, Vec<f64>>,
    #[serde(default)]
    pub parameters: HashMap<String, serde_json::Value>,
    pub origin: Origin,
    pub primary_pos_source: String,
}

impl Flight {
    pub fn len(&self) -> usize {
        self.data
            .get("time_flight")
            .map(|v| v.len())
            .unwrap_or(0)
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn col(&self, name: &str) -> Option<&[f64]> {
        self.data.get(name).map(|v| v.as_slice())
    }

    pub fn contains_group(&self, prefix: &str, suffixes: &[&str]) -> bool {
        suffixes
            .iter()
            .all(|s| self.data.contains_key(&format!("{}_{}", prefix, s)))
    }

    pub fn time_flight(&self) -> &[f64] {
        self.col("time_flight").unwrap_or(&[])
    }

    pub fn pos_gps(&self) -> Option<GPS> {
        if self.contains_group("pos", &["latitude", "longitude", "altitude"]) {
            Some(GPS::from_columns(
                self.col("pos_latitude")?,
                self.col("pos_longitude")?,
                self.col("pos_altitude")?,
            ))
        } else if self.contains_group("gps", &["latitude", "longitude", "altitude"]) {
            Some(GPS::from_columns(
                self.col("gps_latitude")?,
                self.col("gps_longitude")?,
                self.col("gps_altitude")?,
            ))
        } else {
            None
        }
    }

    pub fn position_ned(&self) -> Option<Point> {
        let mut n_col = self.col("position_N");
        let mut e_col = self.col("position_E");
        let mut d_col = self.col("position_D");
        if n_col.is_none() {
            n_col = self.col("xkf_N_0");
            e_col = self.col("xkf_E_0");
            d_col = self.col("xkf_D_0");
        }
        Some(Point::from_columns(n_col?, e_col?, d_col?))
    }

    pub fn attitude_euler(&self) -> Option<Point> {
        Some(Point::from_columns(
            self.col("attitude_roll")?,
            self.col("attitude_pitch")?,
            self.col("attitude_yaw")?,
        ))
    }

    pub fn velocity_ned(&self) -> Option<Point> {
        Some(Point::from_columns(
            self.col("velocity_N")?,
            self.col("velocity_E")?,
            self.col("velocity_D")?,
        ))
    }

    pub fn axisrate(&self) -> Option<Point> {
        Some(Point::from_columns(
            self.col("axisrate_roll")?,
            self.col("axisrate_pitch")?,
            self.col("axisrate_yaw")?,
        ))
    }

    pub fn acceleration(&self) -> Option<Point> {
        Some(Point::from_columns(
            self.col("acceleration_x")?,
            self.col("acceleration_y")?,
            self.col("acceleration_z")?,
        ))
    }

    pub fn from_json_str(json_str: &str) -> Result<Self, String> {
        serde_json::from_str(json_str).map_err(|e| e.to_string())
    }

    pub fn from_json_file(path: impl AsRef<Path>) -> Result<Self, String> {
        let content = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        Self::from_json_str(&content)
    }

    /// Construct a `Flight` from a parsed `FCJ` (Flight Coach JSON).
    pub fn from_fc_json(fcj: &FCJ) -> Self {
        let origin = Origin::from_fcjson_parameters(&fcj.parameters);
        let shift = origin.rotation().transform_point(&Point::new(
            fcj.parameters.moveEast,
            -fcj.parameters.moveNorth,
            0.0,
        ));
        let sx = shift.x0();
        let sy = shift.y0();
        let sz = shift.z0();

        let n = fcj.data.len();
        let mut time_flight = Vec::with_capacity(n);
        let mut pos_n = Vec::with_capacity(n);
        let mut pos_e = Vec::with_capacity(n);
        let mut pos_d = Vec::with_capacity(n);
        let mut vn = Vec::with_capacity(n);
        let mut ve = Vec::with_capacity(n);
        let mut vd = Vec::with_capacity(n);
        let mut roll = Vec::with_capacity(n);
        let mut pitch = Vec::with_capacity(n);
        let mut yaw = Vec::with_capacity(n);
        let mut wn = Vec::with_capacity(n);
        let mut we = Vec::with_capacity(n);

        for row in &fcj.data {
            time_flight.push((row.time as f64) * 1e-6);
            pos_n.push(row.N + sx);
            pos_e.push(row.E + sy);
            pos_d.push(row.D + sz);
            vn.push(row.VN.unwrap_or(0.0));
            ve.push(row.VE.unwrap_or(0.0));
            vd.push(row.VD.unwrap_or(0.0));
            roll.push(row.r.to_radians());
            pitch.push(row.p.to_radians());
            yaw.push(row.yw.to_radians());
            wn.push(row.wN.unwrap_or(0.0));
            we.push(row.wE.unwrap_or(0.0));
        }

        let mut data = IndexMap::new();
        data.insert("time_flight".to_string(), time_flight.clone());
        data.insert("time_actual".to_string(), time_flight);
        data.insert("position_N".to_string(), pos_n);
        data.insert("position_E".to_string(), pos_e);
        data.insert("position_D".to_string(), pos_d);
        data.insert("velocity_N".to_string(), vn);
        data.insert("velocity_E".to_string(), ve);
        data.insert("velocity_D".to_string(), vd);
        data.insert("attitude_roll".to_string(), roll);
        data.insert("attitude_pitch".to_string(), pitch);
        data.insert("attitude_yaw".to_string(), yaw);
        data.insert("wind_N".to_string(), wn);
        data.insert("wind_E".to_string(), we);

        Self {
            data,
            parameters: HashMap::new(),
            origin,
            primary_pos_source: "xkf_c0".to_string(),
        }
    }

    /// Construct a `Flight` from an ArduPilot `.BIN` log file or parsed `Ardupilot` structure.
    pub fn from_log_file(
        bin_path: impl AsRef<Path>,
        ppsource: &str,
        imu_instance: i64,
    ) -> Result<Self, String> {
        let req_types = [
            "ATT", "POS", "GPS", "XKF1", "XKF2", "NKF1", "NKF2", "IMU", "ORGN", "BARO", "ARSP",
            "RCIN", "RCOU", "MODE", "PARM",
        ];
        let log = Ardupilot::parse_file(bin_path, Some(&req_types), None, false)?;
        Self::from_ardupilot(&log, ppsource, imu_instance)
    }

    pub fn from_ardupilot(
        log: &Ardupilot,
        ppsource: &str,
        imu_instance: i64,
    ) -> Result<Self, String> {
        let att = log
            .table("ATT")
            .ok_or_else(|| "ArduPilot log missing ATT message table".to_string())?;
        let base_ts = att
            .get_f64_col("timestamp")
            .ok_or_else(|| "ATT missing timestamp".to_string())?;
        let base_tus = att
            .get_f64_col("TimeUS")
            .ok_or_else(|| "ATT missing TimeUS".to_string())?;
        if base_ts.is_empty() {
            return Err("ATT table is empty".to_string());
        }

        let time_flight: Vec<f64> = base_tus.iter().map(|&u| u * 1e-6).collect();
        let time_actual: Vec<f64> = base_ts.to_vec();

        let mut data: IndexMap<String, Vec<f64>> = IndexMap::new();
        data.insert("time_flight".to_string(), time_flight.clone());
        data.insert("time_actual".to_string(), time_actual.clone());

        let to_rad = |col: &[f64]| -> Vec<f64> { col.iter().map(|&d| d.to_radians()).collect() };
        if let Some(c) = att.get_f64_col("Roll") {
            data.insert("attitude_roll".to_string(), to_rad(c));
        }
        if let Some(c) = att.get_f64_col("Pitch") {
            data.insert("attitude_pitch".to_string(), to_rad(c));
        }
        if let Some(c) = att.get_f64_col("Yaw") {
            data.insert("attitude_yaw".to_string(), to_rad(c));
        }
        if let Some(c) = att.get_f64_col("DesRoll") {
            data.insert("attdes_roll".to_string(), to_rad(c));
        }
        if let Some(c) = att.get_f64_col("DesPitch") {
            data.insert("attdes_pitch".to_string(), to_rad(c));
        }
        if let Some(c) = att.get_f64_col("DesYaw") {
            data.insert("attdes_yaw".to_string(), to_rad(c));
        }

        // Merge POS
        if let Some(pos_tab) = log.table("POS") {
            if let Some(ts) = pos_tab.get_f64_col("timestamp") {
                if let Some(lat) = pos_tab.get_f64_col("Lat") {
                    data.insert("pos_latitude".to_string(), merge_asof(&time_actual, ts, lat));
                }
                if let Some(lng) = pos_tab.get_f64_col("Lng") {
                    data.insert("pos_longitude".to_string(), merge_asof(&time_actual, ts, lng));
                }
                if let Some(alt) = pos_tab.get_f64_col("Alt") {
                    data.insert("pos_altitude".to_string(), merge_asof(&time_actual, ts, alt));
                }
            }
        }

        // Merge GPS
        if let Some(gps_tab) = log.table("GPS") {
            if let Some(ts) = gps_tab.get_f64_col("timestamp") {
                if let Some(lat) = gps_tab.get_f64_col("Lat") {
                    data.insert("gps_latitude".to_string(), merge_asof(&time_actual, ts, lat));
                }
                if let Some(lng) = gps_tab.get_f64_col("Lng") {
                    data.insert("gps_longitude".to_string(), merge_asof(&time_actual, ts, lng));
                }
                if let Some(alt) = gps_tab.get_f64_col("Alt") {
                    data.insert("gps_altitude".to_string(), merge_asof(&time_actual, ts, alt));
                }
            }
        }

        // EKF1 (XKF1 or NKF1)
        let ekf1_name = if log.table("XKF1").is_some() {
            "XKF1"
        } else {
            "NKF1"
        };
        if let Some(ekf1_raw) = log.table(ekf1_name) {
            let ekf1 = if ekf1_raw.get_f64_col("C").is_some() {
                ekf1_raw.filter_by_int_col("C", imu_instance)
            } else {
                ekf1_raw.clone()
            };
            if let Some(ts) = ekf1.get_f64_col("timestamp") {
                let sample_rate = estimate_sample_rate(ts);
                if let Some(pn) = ekf1.get_f64_col("PN") {
                    data.insert("position_N".to_string(), merge_asof(&time_actual, ts, pn));
                }
                if let Some(pe) = ekf1.get_f64_col("PE") {
                    data.insert("position_E".to_string(), merge_asof(&time_actual, ts, pe));
                }
                if let Some(pd) = ekf1.get_f64_col("PD") {
                    data.insert("position_D".to_string(), merge_asof(&time_actual, ts, pd));
                }
                if let Some(vn) = ekf1.get_f64_col("VN") {
                    let filt = butterworth_lowpass_filtfilt(vn, 5.0, sample_rate, 2);
                    data.insert("velocity_N".to_string(), merge_asof(&time_actual, ts, &filt));
                }
                if let Some(ve) = ekf1.get_f64_col("VE") {
                    let filt = butterworth_lowpass_filtfilt(ve, 5.0, sample_rate, 2);
                    data.insert("velocity_E".to_string(), merge_asof(&time_actual, ts, &filt));
                }
                if let Some(vd) = ekf1.get_f64_col("VD") {
                    let filt = butterworth_lowpass_filtfilt(vd, 5.0, sample_rate, 2);
                    data.insert("velocity_D".to_string(), merge_asof(&time_actual, ts, &filt));
                }
            }
        }

        // EKF2 (XKF2 or NKF2)
        let ekf2_name = if log.table("XKF2").is_some() {
            "XKF2"
        } else {
            "NKF2"
        };
        if let Some(ekf2_raw) = log.table(ekf2_name) {
            let ekf2 = if ekf2_raw.get_f64_col("C").is_some() {
                ekf2_raw.filter_by_int_col("C", imu_instance)
            } else {
                ekf2_raw.clone()
            };
            if let Some(ts) = ekf2.get_f64_col("timestamp") {
                if let Some(vwn) = ekf2.get_f64_col("VWN") {
                    data.insert("wind_N".to_string(), merge_asof(&time_actual, ts, vwn));
                }
                if let Some(vwe) = ekf2.get_f64_col("VWE") {
                    data.insert("wind_E".to_string(), merge_asof(&time_actual, ts, vwe));
                }
            }
        }

        // IMU
        if let Some(imu_raw) = log.table("IMU") {
            let imu = if imu_raw.get_f64_col("I").is_some() {
                imu_raw.filter_by_int_col("I", imu_instance)
            } else {
                imu_raw.clone()
            };
            if let Some(ts) = imu.get_f64_col("timestamp") {
                let sample_rate = estimate_sample_rate(ts);
                for (src_col, dst_col) in [
                    ("GyrX", "axisrate_roll"),
                    ("GyrY", "axisrate_pitch"),
                    ("GyrZ", "axisrate_yaw"),
                    ("AccX", "acceleration_x"),
                    ("AccY", "acceleration_y"),
                    ("AccZ", "acceleration_z"),
                ] {
                    if let Some(col) = imu.get_f64_col(src_col) {
                        let filt = butterworth_lowpass_filtfilt(col, 10.0, sample_rate, 2);
                        data.insert(dst_col.to_string(), merge_asof(&time_actual, ts, &filt));
                    }
                }
            }
        }

        // BARO
        if let Some(baro) = log.table("BARO") {
            if let Some(ts) = baro.get_f64_col("timestamp") {
                if let Some(p) = baro.get_f64_col("Press") {
                    data.insert("air_pressure".to_string(), merge_asof(&time_actual, ts, p));
                }
                if let Some(t) = baro.get_f64_col("Temp") {
                    data.insert("air_temperature".to_string(), merge_asof(&time_actual, ts, t));
                }
            }
        }

        // RCIN
        if let Some(rcin) = log.table("RCIN") {
            if let Some(ts) = rcin.get_f64_col("timestamp") {
                for ch in 1..=8 {
                    let cname = format!("C{}", ch);
                    if let Some(col) = rcin.get_f64_col(&cname) {
                        data.insert(
                            format!("rcin_c{}", ch - 1),
                            merge_asof(&time_actual, ts, col),
                        );
                    }
                }
            }
        }

        // MODE
        if let Some(mode_tab) = log.table("MODE") {
            if let Some(ts) = mode_tab.get_f64_col("timestamp") {
                if let Some(m) = mode_tab.get_f64_col("Mode") {
                    data.insert("flightmode_a".to_string(), merge_asof(&time_actual, ts, m));
                }
            }
        }

        // Origin from ORGN or initial POS/GPS
        let origin = if let Some(orgn) = log.table("ORGN") {
            let lat = orgn.get_f64_col("Lat").and_then(|c| c.first()).copied().unwrap_or(0.0);
            let lng = orgn.get_f64_col("Lng").and_then(|c| c.first()).copied().unwrap_or(0.0);
            let alt = orgn.get_f64_col("Alt").and_then(|c| c.first()).copied().unwrap_or(0.0);
            Origin::new("ekf_origin", GPS::new(lat, lng, alt), 0.0)
        } else if let (Some(lat), Some(lng), Some(alt)) = (
            data.get("pos_latitude").and_then(|c| c.first()),
            data.get("pos_longitude").and_then(|c| c.first()),
            data.get("pos_altitude").and_then(|c| c.first()),
        ) {
            Origin::new("ekf_origin", GPS::new(*lat, *lng, *alt), 0.0)
        } else {
            Origin::zero()
        };

        let mut parameters = HashMap::new();
        for (k, pts) in log.parameters() {
            if let Some(first) = pts.first() {
                parameters.insert(k, serde_json::json!(first.value));
            }
        }

        Ok(Self {
            data,
            parameters,
            origin,
            primary_pos_source: format!("{}_c{}", ppsource, imu_instance),
        })
    }

    pub fn remove_time_flutter(&self) -> Self {
        let mut cloned = self.clone();
        let n = self.len();
        if n > 1 {
            for key in ["time_flight", "time_actual"] {
                if let Some(col) = cloned.data.get_mut(key) {
                    let start = col[0];
                    let end = *col.last().unwrap();
                    for (i, v) in col.iter_mut().enumerate() {
                        *v = start + (end - start) * (i as f64) / ((n - 1) as f64);
                    }
                }
            }
        }
        cloned
    }

    pub fn butter_filter(&self, cutoff_hz: f64) -> Self {
        let mut cloned = self.clone();
        let sample_rate = estimate_sample_rate(self.time_flight());
        for (k, col) in &mut cloned.data {
            if k.starts_with("time")
                || k.starts_with("flightmode")
                || k.starts_with("rcin")
                || k.starts_with("rcout")
            {
                continue;
            }
            if k.starts_with("attitude") {
                let unwrapped = unwrap_1d(col, PI);
                let filt = butterworth_lowpass_filtfilt(&unwrapped, cutoff_hz, sample_rate, 2);
                *col = filt.into_iter().map(|v| v.rem_euclid(2.0 * PI)).collect();
            } else {
                *col = butterworth_lowpass_filtfilt(col, cutoff_hz, sample_rate, 2);
            }
        }
        cloned
    }

    pub fn slice_rows(&self, start: usize, stop: usize) -> Self {
        let mut data = IndexMap::new();
        for (k, v) in &self.data {
            let e = stop.min(v.len());
            let s = start.min(e);
            data.insert(k.clone(), v[s..e].to_vec());
        }
        Self {
            data,
            parameters: self.parameters.clone(),
            origin: self.origin.clone(),
            primary_pos_source: self.primary_pos_source.clone(),
        }
    }

    pub fn flying_only(&self, min_alt: f64, min_vel: f64) -> Self {
        let n = self.len();
        if n == 0 {
            return self.clone();
        }
        let alt_col = self
            .col("gps_altitude")
            .or_else(|| self.col("pos_altitude"));
        let vel = self.velocity_ned();
        let (Some(alts), Some(vels)) = (alt_col, vel) else {
            return self.clone();
        };
        let vnorms = vels.norms();
        let mut first_idx = None;
        let mut last_idx = None;
        for i in 0..n {
            if alts[i] >= min_alt && vnorms[i] > min_vel {
                if first_idx.is_none() {
                    first_idx = Some(i);
                }
                last_idx = Some(i);
            }
        }
        if let (Some(s), Some(e)) = (first_idx, last_idx) {
            self.slice_rows(s, e + 1)
        } else {
            self.clone()
        }
    }
}

fn estimate_sample_rate(ts: &[f64]) -> f64 {
    if ts.len() <= 1 {
        return 25.0;
    }
    let dur = ts.last().unwrap() - ts[0];
    if dur <= 1e-9 {
        25.0
    } else {
        (ts.len() as f64) / dur
    }
}

/// Nearest-timestamp `merge_asof` matching pandas `merge_asof(..., direction="nearest")`.
fn merge_asof(target_ts: &[f64], src_ts: &[f64], src_vals: &[f64]) -> Vec<f64> {
    if src_ts.is_empty() || src_vals.is_empty() {
        return vec![0.0; target_ts.len()];
    }
    let mut out = Vec::with_capacity(target_ts.len());
    for &t in target_ts {
        match src_ts.binary_search_by(|v| v.partial_cmp(&t).unwrap_or(std::cmp::Ordering::Equal)) {
            Ok(idx) => out.push(src_vals[idx]),
            Err(idx) => {
                if idx == 0 {
                    out.push(src_vals[0]);
                } else if idx >= src_ts.len() {
                    out.push(*src_vals.last().unwrap());
                } else {
                    let prev_diff = (t - src_ts[idx - 1]).abs();
                    let next_diff = (src_ts[idx] - t).abs();
                    if prev_diff <= next_diff {
                        out.push(src_vals[idx - 1]);
                    } else {
                        out.push(src_vals[idx]);
                    }
                }
            }
        }
    }
    out
}

/// Generate an `Origin` from a single `.BIN` log using an RC channel switch (`rcin_c{channel} >= 1500`).
pub fn origin_from_channel_switch(flight: &Flight, channel: usize) -> Result<Origin, String> {
    let col_name = format!("rcin_c{}", channel.saturating_sub(1));
    let rcin = flight
        .col(&col_name)
        .ok_or_else(|| format!("Missing column {}", col_name))?;
    let tf = flight.time_flight();
    let gps = flight
        .pos_gps()
        .ok_or_else(|| "Flight missing GPS/POS coordinates".to_string())?;

    let active_indices: Vec<usize> = rcin
        .iter()
        .enumerate()
        .filter_map(|(i, &v)| if v >= 1500.0 { Some(i) } else { None })
        .collect();
    if active_indices.is_empty() {
        return Err("No samples with RC switch >= 1500".to_string());
    }

    let mut groups: Vec<Vec<usize>> = Vec::new();
    let mut cur_group = vec![active_indices[0]];
    for &idx in &active_indices[1..] {
        let prev = *cur_group.last().unwrap();
        if tf[idx] - tf[prev] > 1.0 {
            groups.push(cur_group);
            cur_group = vec![idx];
        } else {
            cur_group.push(idx);
        }
    }
    groups.push(cur_group);

    if groups.len() < 2 {
        return Err("Expected at least 2 switch activations (pilot and centre)".to_string());
    }
    let pilot_idx = *groups[0].last().unwrap();
    let centre_idx = *groups[1].last().unwrap();
    Ok(Origin::from_points(
        "new",
        &gps.row(pilot_idx),
        &gps.row(centre_idx),
    ))
}

/// Generate an `Origin` from separate pilot and centre `Flight` logs.
pub fn origin_from_pilot_centre_logs(pilot: &Flight, centre: &Flight) -> Result<Origin, String> {
    let pgps = pilot
        .pos_gps()
        .ok_or_else(|| "Pilot log missing GPS".to_string())?;
    let cgps = centre
        .pos_gps()
        .ok_or_else(|| "Centre log missing GPS".to_string())?;
    Ok(Origin::from_points(
        "new",
        &pgps.row(pgps.len() - 1),
        &cgps.row(cgps.len() - 1),
    ))
}
