use indexmap::IndexMap;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::path::Path;

pub const HEAD_BYTE1: u8 = 0xA3;
pub const HEAD_BYTE2: u8 = 0x95;
pub const FMT_MSG_TYPE: u8 = 0x80; // 128

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FmtDefinition {
    pub msg_type: u8,
    pub length: usize,
    pub name: String,
    pub format: String,
    pub columns: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ColumnData {
    Float(Vec<f64>),
    String(Vec<String>),
}

impl ColumnData {
    pub fn len(&self) -> usize {
        match self {
            Self::Float(v) => v.len(),
            Self::String(v) => v.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn as_floats(&self) -> Option<&[f64]> {
        match self {
            Self::Float(v) => Some(v),
            Self::String(_) => None,
        }
    }

    pub fn as_strings(&self) -> Option<&[String]> {
        match self {
            Self::String(v) => Some(v),
            Self::Float(_) => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LogTable {
    pub name: String,
    pub columns: IndexMap<String, ColumnData>,
}

impl LogTable {
    pub fn new(name: impl Into<String>, fmt: &FmtDefinition) -> Self {
        let mut columns = IndexMap::new();
        columns.insert("timestamp".to_string(), ColumnData::Float(Vec::new()));
        for (ch, col_name) in fmt.format.chars().zip(fmt.columns.iter()) {
            let is_str = matches!(ch, 'n' | 'N' | 'Z');
            if is_str {
                columns.insert(col_name.clone(), ColumnData::String(Vec::new()));
            } else {
                columns.insert(col_name.clone(), ColumnData::Float(Vec::new()));
            }
        }
        Self {
            name: name.into(),
            columns,
        }
    }

    pub fn len(&self) -> usize {
        self.columns
            .values()
            .next()
            .map(|c| c.len())
            .unwrap_or(0)
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn get_f64_col(&self, col: &str) -> Option<&[f64]> {
        self.columns.get(col).and_then(|c| c.as_floats())
    }

    pub fn get_str_col(&self, col: &str) -> Option<&[String]> {
        self.columns.get(col).and_then(|c| c.as_strings())
    }

    /// Filter rows where integer column `col` equals `val`.
    pub fn filter_by_int_col(&self, col: &str, val: i64) -> Self {
        let Some(filter_col) = self.get_f64_col(col) else {
            return self.clone();
        };
        let indices: Vec<usize> = filter_col
            .iter()
            .enumerate()
            .filter_map(|(i, &v)| if (v.round() as i64) == val { Some(i) } else { None })
            .collect();
        let mut new_cols = IndexMap::new();
        for (k, v) in &self.columns {
            match v {
                ColumnData::Float(vec) => {
                    new_cols.insert(
                        k.clone(),
                        ColumnData::Float(indices.iter().map(|&i| vec[i]).collect()),
                    );
                }
                ColumnData::String(vec) => {
                    new_cols.insert(
                        k.clone(),
                        ColumnData::String(indices.iter().map(|&i| vec[i].clone()).collect()),
                    );
                }
            }
        }
        Self {
            name: self.name.clone(),
            columns: new_cols,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParameterPoint {
    pub timestamp: f64,
    pub time_us: f64,
    pub value: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Ardupilot {
    pub filename: String,
    pub dfs: IndexMap<String, LogTable>,
}

/// Convert GPS week (`GWk`) and GPS milliseconds (`GMS`) to Unix epoch seconds (with 18 leap seconds).
pub fn gps_time_to_unix_time(week: f64, msec: f64) -> f64 {
    let epoch: f64 = 315_964_800.0;
    epoch + 86_400.0 * 7.0 * week + 0.001 * msec - 18.0
}

fn glob_match(pattern: &str, text: &str) -> bool {
    if pattern == "*" {
        return true;
    }
    let p: Vec<char> = pattern.chars().collect();
    let t: Vec<char> = text.chars().collect();
    let (mut pi, mut ti) = (0usize, 0usize);
    let (mut star_pi, mut star_ti) = (None, 0usize);

    while ti < t.len() {
        if pi < p.len() && (p[pi] == '?' || p[pi] == t[ti]) {
            pi += 1;
            ti += 1;
        } else if pi < p.len() && p[pi] == '*' {
            star_pi = Some(pi);
            star_ti = ti;
            pi += 1;
        } else if let Some(sp) = star_pi {
            pi = sp + 1;
            star_ti += 1;
            ti = star_ti;
        } else {
            return false;
        }
    }
    while pi < p.len() && p[pi] == '*' {
        pi += 1;
    }
    pi == p.len()
}

impl Ardupilot {
    pub fn process_patterns(
        available: &[String],
        types: Option<&[String]>,
        nottypes: Option<&[String]>,
    ) -> HashSet<String> {
        let mut matched = HashSet::new();
        match types {
            Some(pats) => {
                for pat in pats {
                    for av in available {
                        if glob_match(pat, av) {
                            matched.insert(av.clone());
                        }
                    }
                }
            }
            None => {
                for av in available {
                    matched.insert(av.clone());
                }
            }
        }
        // Always include PARM if available
        if available.iter().any(|a| a == "PARM") {
            matched.insert("PARM".to_string());
        }
        if let Some(not_pats) = nottypes {
            for pat in not_pats {
                matched.retain(|m| !glob_match(pat, m));
            }
        }
        matched
    }

    pub fn parse_file(
        bin_file: impl AsRef<Path>,
        types: Option<&[&str]>,
        nottypes: Option<&[&str]>,
        zero_time_base: bool,
    ) -> Result<Self, String> {
        let path = bin_file.as_ref();
        let bytes = std::fs::read(path)
            .map_err(|e| format!("Failed to read {}: {}", path.display(), e))?;
        let filename = path.to_string_lossy().to_string();
        Self::parse_bytes(&bytes, filename, types, nottypes, zero_time_base)
    }

    pub fn parse_bytes(
        bytes: &[u8],
        filename: String,
        types: Option<&[&str]>,
        nottypes: Option<&[&str]>,
        zero_time_base: bool,
    ) -> Result<Self, String> {
        // Pass 1: Scan FMT definitions and determine time base (first TimeUS and GPS boot time)
        let mut fmts: HashMap<u8, FmtDefinition> = HashMap::new();
        let mut available_names: Vec<String> = Vec::new();
        let mut first_time_us: Option<f64> = None;
        let mut boot_time_unix: Option<f64> = None;

        let mut i = 0usize;
        while i + 3 <= bytes.len() {
            if bytes[i] == HEAD_BYTE1 && bytes[i + 1] == HEAD_BYTE2 {
                let msg_type = bytes[i + 2];
                if msg_type == FMT_MSG_TYPE {
                    if i + 89 <= bytes.len() {
                        let payload = &bytes[i + 3..i + 89];
                        let def_type = payload[0];
                        let def_len = payload[1] as usize;
                        let name = extract_ascii(&payload[2..6]);
                        let format = extract_ascii(&payload[6..22]);
                        let cols_str = extract_ascii(&payload[22..86]);
                        let columns: Vec<String> = if cols_str.is_empty() {
                            Vec::new()
                        } else {
                            cols_str.split(',').map(|s| s.to_string()).collect()
                        };
                        if !name.is_empty() && def_len >= 3 {
                            if !available_names.contains(&name) {
                                available_names.push(name.clone());
                            }
                            fmts.insert(
                                def_type,
                                FmtDefinition {
                                    msg_type: def_type,
                                    length: def_len,
                                    name,
                                    format,
                                    columns,
                                },
                            );
                        }
                        i += 89;
                        continue;
                    } else {
                        break;
                    }
                } else if let Some(fmt) = fmts.get(&msg_type) {
                    let pkt_len = fmt.length;
                    if pkt_len >= 3 && i + pkt_len <= bytes.len() {
                        let payload = &bytes[i + 3..i + pkt_len];
                        if fmt.format.starts_with('Q') && payload.len() >= 8 {
                            let tus = u64::from_le_bytes(payload[0..8].try_into().unwrap()) as f64;
                            if first_time_us.is_none() && tus > 0.0 {
                                first_time_us = Some(tus);
                            }
                            if boot_time_unix.is_none() && fmt.name == "GPS" && payload.len() >= 16 {
                                // GPS fmt=QBIHBcLLeffffB cols=TimeUS,Status,GMS,GWk,...
                                let status = payload[8];
                                let gms = u32::from_le_bytes(payload[9..13].try_into().unwrap()) as f64;
                                let gwk = u16::from_le_bytes(payload[13..15].try_into().unwrap()) as f64;
                                if status >= 3 && gwk > 1000.0 {
                                    let unix_t = gps_time_to_unix_time(gwk, gms);
                                    boot_time_unix = Some(unix_t - tus * 1e-6);
                                }
                            }
                        }
                        i += pkt_len;
                        continue;
                    }
                }
            }
            i += 1;
        }

        let types_vec: Option<Vec<String>> =
            types.map(|s| s.iter().map(|v| (*v).to_string()).collect());
        let nottypes_vec: Option<Vec<String>> =
            nottypes.map(|s| s.iter().map(|v| (*v).to_string()).collect());
        let match_types = Self::process_patterns(
            &available_names,
            types_vec.as_deref(),
            nottypes_vec.as_deref(),
        );

        let base_t_us = first_time_us.unwrap_or(0.0);
        let boot_unix = boot_time_unix.unwrap_or(0.0);

        // Pass 2: Decode all matching messages
        let mut dfs: IndexMap<String, LogTable> = IndexMap::new();
        let mut last_timestamp = 0.0;
        let mut pos = 0usize;
        while pos + 3 <= bytes.len() {
            if bytes[pos] == HEAD_BYTE1 && bytes[pos + 1] == HEAD_BYTE2 {
                let msg_type = bytes[pos + 2];
                if msg_type == FMT_MSG_TYPE {
                    if pos + 89 <= bytes.len() {
                        if match_types.contains("FMT") {
                            if let Some(fmt_def) = fmts.get(&FMT_MSG_TYPE) {
                                let table = dfs
                                    .entry("FMT".to_string())
                                    .or_insert_with(|| LogTable::new("FMT", fmt_def));
                                decode_into_table(
                                    table,
                                    fmt_def,
                                    &bytes[pos + 3..pos + 89],
                                    zero_time_base,
                                    base_t_us,
                                    boot_unix,
                                    &mut last_timestamp,
                                );
                            }
                        }
                        pos += 89;
                        continue;
                    } else {
                        break;
                    }
                } else if let Some(fmt_def) = fmts.get(&msg_type) {
                    let pkt_len = fmt_def.length;
                    if pkt_len >= 3 && pos + pkt_len <= bytes.len() {
                        if match_types.contains(&fmt_def.name) {
                            let table = dfs
                                .entry(fmt_def.name.clone())
                                .or_insert_with(|| LogTable::new(&fmt_def.name, fmt_def));
                            decode_into_table(
                                table,
                                fmt_def,
                                &bytes[pos + 3..pos + pkt_len],
                                zero_time_base,
                                base_t_us,
                                boot_unix,
                                &mut last_timestamp,
                            );
                        }
                        pos += pkt_len;
                        continue;
                    }
                }
            }
            pos += 1;
        }

        Ok(Self { filename, dfs })
    }

    pub fn table(&self, name: &str) -> Option<&LogTable> {
        self.dfs.get(name)
    }

    pub fn parameters(&self) -> IndexMap<String, Vec<ParameterPoint>> {
        let mut result: IndexMap<String, Vec<ParameterPoint>> = IndexMap::new();
        let Some(parm) = self.dfs.get("PARM") else {
            return result;
        };
        let Some(ts) = parm.get_f64_col("timestamp") else {
            return result;
        };
        let Some(tus) = parm.get_f64_col("TimeUS") else {
            return result;
        };
        let Some(names) = parm.get_str_col("Name") else {
            return result;
        };
        let Some(vals) = parm.get_f64_col("Value") else {
            return result;
        };

        for i in 0..parm.len() {
            let entry = result.entry(names[i].clone()).or_default();
            let should_add = match entry.last() {
                None => true,
                Some(prev) => (vals[i] - prev.value).abs() > 0.0,
            };
            if should_add {
                entry.push(ParameterPoint {
                    timestamp: ts[i],
                    time_us: tus[i],
                    value: vals[i],
                });
            }
        }
        result
    }
}

fn extract_ascii(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).trim().to_string()
}

fn decode_into_table(
    table: &mut LogTable,
    fmt: &FmtDefinition,
    payload: &[u8],
    zero_time_base: bool,
    base_t_us: f64,
    boot_unix: f64,
    last_timestamp: &mut f64,
) {
    let mut offset = 0usize;
    let mut row_time_us: Option<f64> = None;

    for (ch, col_name) in fmt.format.chars().zip(fmt.columns.iter()) {
        match ch {
            'b' => {
                if offset + 1 > payload.len() {
                    return;
                }
                let val = (payload[offset] as i8) as f64;
                offset += 1;
                push_f64(table, col_name, val);
            }
            'B' | 'M' => {
                if offset + 1 > payload.len() {
                    return;
                }
                let val = payload[offset] as f64;
                offset += 1;
                push_f64(table, col_name, val);
            }
            'h' => {
                if offset + 2 > payload.len() {
                    return;
                }
                let val = i16::from_le_bytes(payload[offset..offset + 2].try_into().unwrap()) as f64;
                offset += 2;
                push_f64(table, col_name, val);
            }
            'H' => {
                if offset + 2 > payload.len() {
                    return;
                }
                let val = u16::from_le_bytes(payload[offset..offset + 2].try_into().unwrap()) as f64;
                offset += 2;
                push_f64(table, col_name, val);
            }
            'c' => {
                if offset + 2 > payload.len() {
                    return;
                }
                let val =
                    (i16::from_le_bytes(payload[offset..offset + 2].try_into().unwrap()) as f64)
                        * 0.01;
                offset += 2;
                push_f64(table, col_name, val);
            }
            'C' => {
                if offset + 2 > payload.len() {
                    return;
                }
                let val =
                    (u16::from_le_bytes(payload[offset..offset + 2].try_into().unwrap()) as f64)
                        * 0.01;
                offset += 2;
                push_f64(table, col_name, val);
            }
            'i' => {
                if offset + 4 > payload.len() {
                    return;
                }
                let val = i32::from_le_bytes(payload[offset..offset + 4].try_into().unwrap()) as f64;
                offset += 4;
                push_f64(table, col_name, val);
            }
            'I' => {
                if offset + 4 > payload.len() {
                    return;
                }
                let val = u32::from_le_bytes(payload[offset..offset + 4].try_into().unwrap()) as f64;
                offset += 4;
                push_f64(table, col_name, val);
            }
            'e' => {
                if offset + 4 > payload.len() {
                    return;
                }
                let val =
                    (i32::from_le_bytes(payload[offset..offset + 4].try_into().unwrap()) as f64)
                        * 0.01;
                offset += 4;
                push_f64(table, col_name, val);
            }
            'E' => {
                if offset + 4 > payload.len() {
                    return;
                }
                let val =
                    (u32::from_le_bytes(payload[offset..offset + 4].try_into().unwrap()) as f64)
                        * 0.01;
                offset += 4;
                push_f64(table, col_name, val);
            }
            'L' => {
                if offset + 4 > payload.len() {
                    return;
                }
                let val =
                    (i32::from_le_bytes(payload[offset..offset + 4].try_into().unwrap()) as f64)
                        * 1.0e-7;
                offset += 4;
                push_f64(table, col_name, val);
            }
            'f' => {
                if offset + 4 > payload.len() {
                    return;
                }
                let val = f32::from_le_bytes(payload[offset..offset + 4].try_into().unwrap()) as f64;
                offset += 4;
                push_f64(table, col_name, val);
            }
            'd' => {
                if offset + 8 > payload.len() {
                    return;
                }
                let val = f64::from_le_bytes(payload[offset..offset + 8].try_into().unwrap());
                offset += 8;
                push_f64(table, col_name, val);
            }
            'q' => {
                if offset + 8 > payload.len() {
                    return;
                }
                let val = i64::from_le_bytes(payload[offset..offset + 8].try_into().unwrap()) as f64;
                offset += 8;
                push_f64(table, col_name, val);
            }
            'Q' => {
                if offset + 8 > payload.len() {
                    return;
                }
                let val = u64::from_le_bytes(payload[offset..offset + 8].try_into().unwrap()) as f64;
                offset += 8;
                if col_name == "TimeUS" {
                    row_time_us = Some(val);
                }
                push_f64(table, col_name, val);
            }
            'n' => {
                if offset + 4 > payload.len() {
                    return;
                }
                let s = extract_ascii(&payload[offset..offset + 4]);
                offset += 4;
                push_str(table, col_name, s);
            }
            'N' => {
                if offset + 16 > payload.len() {
                    return;
                }
                let s = extract_ascii(&payload[offset..offset + 16]);
                offset += 16;
                push_str(table, col_name, s);
            }
            'Z' => {
                if offset + 64 > payload.len() {
                    return;
                }
                let s = extract_ascii(&payload[offset..offset + 64]);
                offset += 64;
                push_str(table, col_name, s);
            }
            'a' => {
                if offset + 64 > payload.len() {
                    return;
                }
                offset += 64;
                push_f64(table, col_name, 0.0);
            }
            _ => {}
        }
    }

    let ts = if let Some(tus) = row_time_us {
        let computed = if zero_time_base {
            (tus - base_t_us) * 1e-6
        } else {
            boot_unix + tus * 1e-6
        };
        *last_timestamp = computed;
        computed
    } else {
        *last_timestamp
    };
    push_f64(table, "timestamp", ts);
}

fn push_f64(table: &mut LogTable, col: &str, val: f64) {
    if let Some(ColumnData::Float(v)) = table.columns.get_mut(col) {
        v.push(val);
    }
}

fn push_str(table: &mut LogTable, col: &str, val: String) {
    if let Some(ColumnData::String(v)) = table.columns.get_mut(col) {
        v.push(val);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn test_inputs_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../packages/ArdupilotLogReader/tests/test_inputs")
    }

    #[test]
    fn test_dfs_parse_bin() {
        let type_request = [
            "XKF1", "XKQ1", "ARSP", "GPS", "RCIN", "RCOU", "IMU", "BARO", "MODE", "RPM", "MAG",
        ];
        let bin_path = test_inputs_dir().join("test_log_00000052.BIN");
        let log = Ardupilot::parse_file(&bin_path, Some(&type_request), None, true)
            .expect("parse test_log_00000052.BIN");

        let mut expected: HashSet<String> = type_request.iter().map(|s| (*s).to_string()).collect();
        expected.insert("PARM".to_string());
        let actual: HashSet<String> = log.dfs.keys().cloned().collect();
        assert_eq!(actual, expected);

        let params = log.parameters();
        assert!(!params.is_empty());
    }
}
