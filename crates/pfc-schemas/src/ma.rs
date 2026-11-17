use crate::fcj::{ManResult, ScoreProperties};
use crate::positioning::{deserialize_opt_enum, Direction};
use crate::sinfo::ScheduleInfo;
use indexmap::IndexMap;
use semver::Version;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FAVersion {
    All,
    Latest,
    Specific(String),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MA {
    pub name: String,
    pub id: i64,
    pub schedule: ScheduleInfo,
    #[serde(default, deserialize_with = "deserialize_opt_enum")]
    pub schedule_direction: Option<Direction>,
    pub flown: serde_json::Value,
    #[serde(default)]
    pub history: Option<IndexMap<String, ManResult>>,
    #[serde(default)]
    pub mdef: Option<serde_json::Value>,
    #[serde(default)]
    pub manoeuvre: Option<serde_json::Value>,
    #[serde(default)]
    pub template: Option<serde_json::Value>,
    #[serde(default)]
    pub templates: Option<serde_json::Value>,
    #[serde(default)]
    pub corrected: Option<serde_json::Value>,
    #[serde(default)]
    pub corrected_template: Option<serde_json::Value>,
    #[serde(default)]
    pub scores: Option<serde_json::Value>,
}

impl MA {
    pub fn k(&self) -> Result<f64, String> {
        let Some(mdef) = &self.mdef else {
            return Err("No k factor available".to_string());
        };
        let target = match mdef {
            serde_json::Value::Object(_) => Some(mdef),
            serde_json::Value::Array(arr) => arr.first(),
            _ => None,
        };
        if let Some(obj) = target {
            if let Some(k) = obj
                .get("info")
                .and_then(|i| i.get("k"))
                .and_then(|v| v.as_f64())
            {
                return Ok(k);
            }
        }
        Err("No k factor available".to_string())
    }

    pub fn latest_version(&self) -> Option<String> {
        let hist = self.history.as_ref()?;
        hist.keys()
            .filter_map(|k| {
                let clean = k.strip_prefix('v').unwrap_or(k);
                Version::parse(clean).ok().map(|v| (v, k.clone()))
            })
            .max_by(|(a, _), (b, _)| a.cmp(b))
            .map(|(_, k)| k)
    }

    pub fn basic(&self, mdef_override: Option<serde_json::Value>) -> Self {
        Self {
            name: self.name.clone(),
            id: self.id,
            schedule: self.schedule.clone(),
            schedule_direction: self.schedule_direction,
            flown: self.flown.clone(),
            history: self.history.clone(),
            mdef: mdef_override.or_else(|| self.mdef.clone()),
            manoeuvre: None,
            template: None,
            templates: None,
            corrected: None,
            corrected_template: None,
            scores: None,
        }
    }

    pub fn k_factored_score(
        &self,
        version: &str,
        props: Option<ScoreProperties>,
    ) -> Result<f64, String> {
        let k = self.k()?;
        let hist = self
            .history
            .as_ref()
            .ok_or_else(|| "No history available".to_string())?;
        let target_ver = if version == "Latest" {
            self.latest_version()
                .ok_or_else(|| "No valid latest version".to_string())?
        } else {
            version.to_string()
        };
        let mr = hist
            .get(&target_ver)
            .ok_or_else(|| format!("Version {} not found in history", target_ver))?;
        let sv = mr
            .get_score(props)
            .ok_or_else(|| "Score properties not found".to_string())?;
        Ok(sv.total * k)
    }

    pub fn simplify_history(&self) -> Self {
        let mut cloned = self.clone();
        if let Some(hist) = &self.history {
            let mut new_hist = IndexMap::new();
            for (k, v) in hist {
                let clean = k.strip_prefix('v').unwrap_or(k).to_string();
                new_hist.insert(clean, v.clone());
            }
            cloned.history = Some(new_hist);
        }
        cloned
    }

    pub fn rename_version(&self, old_v: &str, new_v: &str) -> Self {
        let mut cloned = self.clone();
        if let Some(hist) = &mut cloned.history {
            if let Some(val) = hist.shift_remove(old_v) {
                hist.insert(new_v.to_string(), val);
            }
        }
        cloned
    }
}
