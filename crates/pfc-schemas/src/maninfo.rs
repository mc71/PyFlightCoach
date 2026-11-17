use crate::positioning::{deserialize_opt_enum, BoxLocation, Position};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ManInfo {
    pub name: String,
    pub short_name: String,
    #[serde(default)]
    pub k: f64,
    #[serde(default, deserialize_with = "deserialize_opt_enum")]
    pub position: Option<Position>,
    #[serde(default)]
    pub start: BoxLocation,
    #[serde(default)]
    pub end: BoxLocation,
    #[serde(default)]
    pub centre_points: Vec<i32>,
    #[serde(default)]
    pub centred_els: Vec<(i32, f64)>,
}

impl ManInfo {
    #[allow(clippy::too_many_arguments)]
    pub fn build(
        name: impl Into<String>,
        short_name: impl Into<String>,
        k: f64,
        position: Option<Position>,
        start: BoxLocation,
        end: BoxLocation,
        centre_points: Option<Vec<i32>>,
        centred_els: Option<Vec<(i32, f64)>>,
    ) -> Self {
        Self {
            name: name.into(),
            short_name: short_name.into(),
            k,
            position,
            start,
            end,
            centre_points: centre_points.unwrap_or_default(),
            centred_els: centred_els.unwrap_or_default(),
        }
    }
}
