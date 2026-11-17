use crate::maninfo::ManInfo;
use crate::positioning::Direction;
use crate::sinfo::ScheduleInfo;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MDef {
    pub info: ManInfo,
    pub mps: HashMap<String, serde_json::Value>,
    pub eds: HashMap<String, serde_json::Value>,
    pub box_: Option<HashMap<String, serde_json::Value>>,
    #[serde(rename = "box", default)]
    pub box_data: HashMap<String, serde_json::Value>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MOption {
    pub options: Vec<MDef>,
    #[serde(default)]
    pub active: usize,
}

impl MOption {
    pub fn uid(&self) -> Option<&str> {
        self.options.first().map(|m| m.info.short_name.as_str())
    }

    pub fn info(&self) -> Option<&ManInfo> {
        self.options.get(self.active).map(|m| &m.info)
    }

    pub fn mps(&self) -> Option<&HashMap<String, serde_json::Value>> {
        self.options.get(self.active).map(|m| &m.mps)
    }

    pub fn eds(&self) -> Option<&HashMap<String, serde_json::Value>> {
        self.options.get(self.active).map(|m| &m.eds)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DirectionDefinition {
    pub manid: usize,
    pub direction: Direction,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SDefFile {
    pub category: String,
    pub schedule: String,
    pub fa_version: String,
    pub mdefs: HashMap<String, serde_json::Value>,
}

impl SDefFile {
    pub fn sinfo(&self) -> ScheduleInfo {
        ScheduleInfo::new(&self.category, &self.schedule)
    }
}
