use chrono::NaiveDate;
use regex::Regex;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::collections::HashMap;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum StringOrFloat {
    Float(f64),
    String(String),
}

impl StringOrFloat {
    pub fn as_f64(&self) -> f64 {
        match self {
            Self::Float(v) => *v,
            Self::String(s) => s.trim().parse::<f64>().unwrap_or(0.0),
        }
    }
}

impl Default for StringOrFloat {
    fn default() -> Self {
        Self::Float(0.0)
    }
}

pub fn deserialize_f64_or_str<'de, D>(deserializer: D) -> Result<f64, D::Error>
where
    D: Deserializer<'de>,
{
    let sof = StringOrFloat::deserialize(deserializer)?;
    Ok(sof.as_f64())
}

pub fn serialize_as_str<S>(val: &f64, serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    serializer.serialize_str(&val.to_string())
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct View {
    pub position: HashMap<String, serde_json::Value>,
    pub target: HashMap<String, serde_json::Value>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[allow(non_snake_case)]
pub struct Parameters {
    pub rotation: f64,
    pub start: i64,
    pub stop: i64,
    pub moveEast: f64,
    pub moveNorth: f64,
    pub wingspan: f64,
    pub modelwingspan: f64,
    pub elevate: f64,
    pub originLat: f64,
    pub originLng: f64,
    pub originAlt: f64,
    #[serde(deserialize_with = "deserialize_f64_or_str")]
    pub pilotLat: f64,
    #[serde(deserialize_with = "deserialize_f64_or_str")]
    pub pilotLng: f64,
    #[serde(deserialize_with = "deserialize_f64_or_str")]
    pub pilotAlt: f64,
    #[serde(deserialize_with = "deserialize_f64_or_str")]
    pub centerLat: f64,
    #[serde(deserialize_with = "deserialize_f64_or_str")]
    pub centerLng: f64,
    #[serde(deserialize_with = "deserialize_f64_or_str")]
    pub centerAlt: f64,
    pub schedule: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HumanResult {
    pub name: String,
    pub date: NaiveDate,
    pub scores: Vec<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct El {
    pub name: String,
    pub start: f64,
    pub stop: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ScoreValues {
    pub intra: f64,
    pub inter: f64,
    pub positioning: f64,
    pub total: f64,
}

impl ScoreValues {
    pub fn get_group(&self, group: &str) -> Option<f64> {
        match group {
            "intra" => Some(self.intra),
            "inter" => Some(self.inter),
            "positioning" => Some(self.positioning),
            "total" => Some(self.total),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ScoreProperties {
    #[serde(default = "default_difficulty")]
    pub difficulty: i32,
    #[serde(default)]
    pub truncate: bool,
}

fn default_difficulty() -> i32 {
    3
}

impl Default for ScoreProperties {
    fn default() -> Self {
        Self {
            difficulty: 3,
            truncate: false,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Score {
    pub score: ScoreValues,
    pub properties: ScoreProperties,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ManResult {
    pub els: Vec<El>,
    #[serde(default)]
    pub results: Vec<Score>,
}

impl ManResult {
    pub fn get_score(&self, props: Option<ScoreProperties>) -> Option<&ScoreValues> {
        let target = props.unwrap_or_default();
        self.results
            .iter()
            .find(|r| r.properties == target)
            .map(|r| &r.score)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FCSResult {
    pub fa_version: String,
    pub manresults: Vec<Option<ManResult>>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Man {
    pub name: String,
    #[serde(default)]
    pub k: Option<f64>,
    pub id: String,
    pub sp: i64,
    pub wd: f64,
    pub start: i64,
    pub stop: i64,
    pub sel: bool,
    pub background: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[allow(non_snake_case)]
pub struct Data {
    #[serde(default)]
    pub VN: Option<f64>,
    #[serde(default)]
    pub VE: Option<f64>,
    #[serde(default)]
    pub VD: Option<f64>,
    #[serde(default)]
    pub dPD: Option<f64>,
    pub r: f64,
    pub p: f64,
    pub yw: f64,
    pub N: f64,
    pub E: f64,
    pub D: f64,
    pub time: i64,
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
    #[serde(default)]
    pub wN: Option<f64>,
    #[serde(default)]
    pub wE: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Origin {
    pub lat: f64,
    pub lng: f64,
    pub alt: f64,
    pub heading: f64,
    #[serde(default)]
    pub move_north: f64,
    #[serde(default)]
    pub move_east: f64,
    #[serde(default)]
    pub move_down: f64,
}

impl Origin {
    pub fn zero() -> Self {
        Self {
            lat: 0.0,
            lng: 0.0,
            alt: 0.0,
            heading: 0.0,
            move_north: 0.0,
            move_east: 0.0,
            move_down: 0.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FCJ {
    pub version: String,
    pub comments: String,
    pub name: String,
    pub view: View,
    pub parameters: Parameters,
    pub scored: bool,
    pub scores: Vec<Option<f64>>,
    #[serde(default)]
    pub human_scores: Vec<HumanResult>,
    #[serde(default)]
    pub fcs_scores: Vec<FCSResult>,
    pub mans: Vec<Man>,
    pub data: Vec<Data>,
    #[serde(default)]
    pub jhash: Option<i64>,
}

impl FCJ {
    pub fn id(&self) -> Option<String> {
        let re = Regex::new(r"\d{8}").ok()?;
        re.find(&self.name).map(|m| m.as_str().to_string())
    }

    pub fn created(&self) -> Option<NaiveDate> {
        let re = Regex::new(r"_(\d{2})_(\d{2})_(\d{2})_").ok()?;
        let caps = re.captures(&self.name)?;
        let y: i32 = 2000 + caps.get(1)?.as_str().parse::<i32>().ok()?;
        let m: u32 = caps.get(2)?.as_str().parse::<u32>().ok()?;
        let d: u32 = caps.get(3)?.as_str().parse::<u32>().ok()?;
        NaiveDate::from_ymd_opt(y, m, d)
    }

    pub fn origin(&self) -> Origin {
        Origin {
            lat: self.parameters.pilotLat,
            lng: self.parameters.pilotLng,
            alt: self.parameters.pilotAlt,
            heading: self.parameters.rotation.to_degrees(),
            move_north: self.parameters.moveNorth,
            move_east: self.parameters.moveEast,
            move_down: self.parameters.elevate,
        }
    }

    pub fn latest_version(&self) -> Option<&str> {
        self.fcs_scores
            .iter()
            .map(|r| r.fa_version.as_str())
            .max()
    }

    /// Returns per-manoeuvre (raw_score, k_factored_score) for a given FA version
    /// (excluding takeoff mans[0] and landing mans[last]).
    pub fn version_scores(&self, fa_version: &str, props: Option<ScoreProperties>) -> Vec<(String, f64, f64, f64)> {
        let Some(fcs) = self.fcs_scores.iter().find(|r| r.fa_version == fa_version) else {
            return Vec::new();
        };
        let mut out = Vec::new();
        let active_mans = if self.mans.len() >= 2 {
            &self.mans[1..self.mans.len() - 1]
        } else {
            &[]
        };
        for (i, man) in active_mans.iter().enumerate() {
            if let Some(Some(mr)) = fcs.manresults.get(i + 1) {
                if let Some(sv) = mr.get_score(props) {
                    let k = man.k.unwrap_or(0.0);
                    out.push((man.name.clone(), k, sv.total, sv.total * k));
                }
            }
        }
        out
    }
}
