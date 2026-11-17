use crate::maninfo::ManInfo;
use crate::positioning::Direction;
use serde::{Deserialize, Serialize};

pub fn fcj_categories() -> &'static [(&'static str, &'static str)] {
    &[
        ("F3A FAI", "f3a"),
        ("F3A", "f3a"),
        ("US AMA", "ama"),
        ("F3A UK", "f3auk"),
        ("F3A US", "ama"),
        ("IMAC", "IMAC"),
    ]
}

pub fn fcj_schedules() -> &'static [(&'static str, &'static str)] {
    &[
        ("P23", "p23"),
        ("F23", "f23"),
        ("P25", "p25"),
        ("F25", "f25"),
        ("P27", "p27"),
        ("F27", "f27"),
        ("A27", "a27"),
        ("Unlimited 2024", "unlimited2024"),
        ("Intermediate 24", "intermediate2024"),
    ]
}

pub fn lookup(val: &str, table: &[(&str, &str)]) -> String {
    let normalized = val.replace('_', " ");
    for (k, v) in table {
        if *k == normalized {
            return (*v).to_string();
        }
    }
    normalized
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ManDetails {
    pub name: String,
    pub id: usize,
    pub k: f64,
    pub entry: Option<Direction>,
}

impl ManDetails {
    pub fn from_maninfo(info: &ManInfo, index: usize) -> Self {
        Self {
            name: info.short_name.clone(),
            id: index + 1,
            k: info.k,
            entry: info.start.direction,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScheduleInfo {
    pub category: String,
    pub name: String,
}

impl ScheduleInfo {
    pub fn new(category: impl Into<String>, name: impl Into<String>) -> Self {
        Self {
            category: category.into(),
            name: name.into(),
        }
    }

    pub fn build(category: &str, name: &str) -> Self {
        Self {
            category: category.to_lowercase(),
            name: name.to_lowercase(),
        }
    }

    pub fn from_str_name(fname: &str) -> Self {
        let stem = fname.split('.').next().unwrap_or(fname);
        let parts: Vec<&str> = stem.split('_').collect();
        if parts.len() == 1 {
            Self::new("f3a", parts[0])
        } else {
            Self::new(parts[0], parts[1])
        }
    }

    pub fn mixed() -> Self {
        Self::new("na", "mixed")
    }

    pub fn lookup_category(category: &str) -> String {
        lookup(category, fcj_categories())
    }

    pub fn lookup_schedule(schedule: &str) -> String {
        lookup(schedule, fcj_schedules())
    }

    pub fn fcj_to_pfc(&self) -> Self {
        let cat = Self::lookup_category(&self.category);
        let sch = Self::lookup_schedule(&self.name);
        if cat == "ama" && sch == "Advanced 24" {
            return Self::new("f3a", "a25");
        }
        Self::new(cat, sch)
    }

    pub fn pfc_to_fcj(&self) -> Self {
        let cat = fcj_categories()
            .iter()
            .find(|(_, v)| *v == self.category)
            .map(|(k, _)| (*k).to_string())
            .unwrap_or_else(|| self.category.clone());
        let sch = fcj_schedules()
            .iter()
            .find(|(_, v)| *v == self.name)
            .map(|(k, _)| (*k).to_string())
            .unwrap_or_else(|| self.name.clone());
        Self::new(cat, sch)
    }

    pub fn from_fcj_sch(sch: &[String]) -> Self {
        if sch.len() >= 2 {
            Self::new(&sch[0], &sch[1]).fcj_to_pfc()
        } else if sch.len() == 1 {
            Self::new("f3a", &sch[0]).fcj_to_pfc()
        } else {
            Self::mixed()
        }
    }

    pub fn to_fcj_sch(&self) -> Vec<String> {
        let fcj = self.pfc_to_fcj();
        vec![fcj.category, fcj.name]
    }
}

impl std::fmt::Display for ScheduleInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}_{}", self.category, self.name)
    }
}

impl PartialEq for ScheduleInfo {
    fn eq(&self, other: &Self) -> bool {
        self.fcj_to_pfc().to_string() == other.fcj_to_pfc().to_string()
    }
}

impl Eq for ScheduleInfo {}
