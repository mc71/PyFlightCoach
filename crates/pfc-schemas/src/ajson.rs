use crate::fcj::{Origin, ScoreProperties};
use crate::ma::MA;
use crate::sinfo::ScheduleInfo;
use chrono::{DateTime, Utc};
use semver::Version;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::path::Path;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[allow(non_snake_case)]
pub struct AJson {
    #[serde(default)]
    pub origin: Option<Origin>,
    pub isComp: bool,
    #[serde(default)]
    pub sourceBin: Option<String>,
    #[serde(default)]
    pub sourceFCJ: Option<String>,
    #[serde(default)]
    pub bootTime: Option<DateTime<Utc>>,
    pub mans: Vec<MA>,
}

impl AJson {
    pub fn parse_json_str(json_str: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json_str)
    }

    pub fn from_file(path: impl AsRef<Path>) -> Result<Self, String> {
        let content = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        Self::parse_json_str(&content).map_err(|e| e.to_string())
    }

    pub fn basic(&self, mdefs: Option<&[serde_json::Value]>) -> Self {
        let mans = self
            .mans
            .iter()
            .enumerate()
            .map(|(i, m)| {
                let mdef = mdefs.and_then(|list| list.get(i)).cloned();
                m.basic(mdef)
            })
            .collect();
        Self {
            origin: self.origin,
            isComp: self.isComp,
            sourceBin: self.sourceBin.clone(),
            sourceFCJ: self.sourceFCJ.clone(),
            bootTime: self.bootTime,
            mans,
        }
    }

    pub fn man_names(&self) -> Vec<&str> {
        self.mans.iter().map(|m| m.name.as_str()).collect()
    }

    pub fn get_man_by_index(&self, idx: usize) -> Option<&MA> {
        self.mans.get(idx)
    }

    pub fn get_man_by_name(&self, name: &str) -> Option<&MA> {
        self.mans.iter().find(|m| m.name == name)
    }

    pub fn k_factors(&self) -> Result<Vec<f64>, String> {
        self.mans.iter().map(|m| m.k()).collect()
    }

    pub fn schedule(&self) -> ScheduleInfo {
        let Some(first) = self.mans.first() else {
            return ScheduleInfo::mixed();
        };
        let first_sch = &first.schedule;
        if self.mans.iter().all(|m| &m.schedule == first_sch) {
            first_sch.fcj_to_pfc()
        } else {
            ScheduleInfo::mixed()
        }
    }

    pub fn all_versions(&self) -> Vec<String> {
        let mut set = BTreeSet::new();
        for m in &self.mans {
            if let Some(hist) = &m.history {
                for k in hist.keys() {
                    set.insert(k.clone());
                }
            }
        }
        set.into_iter().collect()
    }

    pub fn all_valid_versions(&self) -> Vec<String> {
        self.all_versions()
            .into_iter()
            .filter(|v| {
                let clean = v.strip_prefix('v').unwrap_or(v);
                Version::parse(clean).is_ok()
            })
            .collect()
    }

    pub fn latest_version(&self) -> Option<String> {
        self.all_valid_versions()
            .into_iter()
            .filter_map(|k| {
                let clean = k.strip_prefix('v').unwrap_or(&k);
                Version::parse(clean).ok().map(|v| (v, k))
            })
            .max_by(|(a, _), (b, _)| a.cmp(b))
            .map(|(_, k)| k)
    }

    pub fn get_scores(
        &self,
        version: &str,
        props: Option<ScoreProperties>,
        group: &str,
    ) -> Result<Vec<f64>, String> {
        let target_ver = if version == "Latest" {
            self.latest_version()
                .ok_or_else(|| "No valid latest version".to_string())?
        } else {
            version.to_string()
        };
        let mut scores = Vec::with_capacity(self.mans.len());
        for m in &self.mans {
            let hist = m
                .history
                .as_ref()
                .ok_or_else(|| format!("Manoeuvre {} has no history", m.name))?;
            let mr = hist.get(&target_ver).ok_or_else(|| {
                format!("Manoeuvre {} missing version {}", m.name, target_ver)
            })?;
            let sv = mr
                .get_score(props)
                .ok_or_else(|| format!("Manoeuvre {} missing score properties", m.name))?;
            let val = sv
                .get_group(group)
                .ok_or_else(|| format!("Unknown score group {}", group))?;
            scores.push(val);
        }
        Ok(scores)
    }

    pub fn total_score(
        &self,
        version: &str,
        props: Option<ScoreProperties>,
        group: &str,
    ) -> Result<f64, String> {
        let scores = self.get_scores(version, props, group)?;
        let ks = self.k_factors()?;
        Ok(scores.iter().zip(ks.iter()).map(|(s, k)| s * k).sum())
    }

    pub fn check_version(&self, version: &str) -> bool {
        let clean = version.strip_prefix('v').unwrap_or(version);
        self.mans.iter().all(|m| {
            m.history
                .as_ref()
                .map(|h| h.contains_key(clean) || h.contains_key(version))
                .unwrap_or(false)
        })
    }

    pub fn rename_version(&self, old_v: &str, new_v: &str) -> Self {
        let mut cloned = self.clone();
        cloned.mans = self
            .mans
            .iter()
            .map(|m| m.rename_version(old_v, new_v))
            .collect();
        cloned
    }
}
