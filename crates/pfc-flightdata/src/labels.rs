use indexmap::IndexMap;
use pfc_geometry::{get_index, get_value, IndexDirection};
use serde::{Deserialize, Serialize};

/// Time interval `[start, stop]` with optional hierarchical sublabels.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Label {
    pub start: f64,
    pub stop: f64,
    #[serde(default)]
    pub sublabels: LabelGroups,
}

impl Label {
    pub fn new(start: f64, stop: f64) -> Self {
        Self {
            start,
            stop,
            sublabels: LabelGroups::default(),
        }
    }

    pub fn with_sublabels(start: f64, stop: f64, sublabels: LabelGroups) -> Self {
        Self {
            start,
            stop,
            sublabels,
        }
    }

    #[inline]
    pub fn width(&self) -> f64 {
        self.stop - self.start
    }

    #[inline]
    pub fn is_valid(&self) -> bool {
        self.start < self.stop
    }

    #[inline]
    pub fn intersects(&self, tstart: f64, tstop: f64) -> bool {
        self.start <= tstop && self.stop > tstart
    }

    pub fn contains(&self, t: f64, inclusive: bool) -> bool {
        if inclusive {
            t >= self.start - 1e-12 && t <= self.stop + 1e-12
        } else {
            t >= self.start - 1e-12 && t < self.stop - 1e-12
        }
    }

    pub fn to_iloc(&self, t: &[f64]) -> Result<Self, String> {
        let s = get_index(t, self.start, IndexDirection::Forward, None)?;
        let e = get_index(t, self.stop, IndexDirection::Backward, None)?;
        Ok(Self::with_sublabels(s, e, self.sublabels.clone()))
    }

    pub fn to_t(&self, t: &[f64]) -> Result<Self, String> {
        let s = get_value(t, self.start)?;
        let e = get_value(t, self.stop)?;
        Ok(Self::with_sublabels(s, e, self.sublabels.clone()))
    }

    pub fn offset(&self, dt: f64) -> Self {
        Self {
            start: self.start + dt,
            stop: self.stop + dt,
            sublabels: self.sublabels.offset(dt),
        }
    }
}

/// Ordered dictionary of contiguous/tesselated `Label`s.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct LabelGroup {
    pub labels: IndexMap<String, Label>,
}

impl LabelGroup {
    pub fn new() -> Self {
        Self {
            labels: IndexMap::new(),
        }
    }

    pub fn from_map(labels: IndexMap<String, Label>) -> Self {
        Self { labels }
    }

    pub fn is_empty(&self) -> bool {
        self.labels.is_empty()
    }

    pub fn len(&self) -> usize {
        self.labels.len()
    }

    pub fn get(&self, name: &str) -> Option<&Label> {
        self.labels.get(name)
    }

    /// Build a tesselated `LabelGroup` from a per-timestep string label array `labels` over time `t`.
    pub fn read_array(t: &[f64], labels: &[String]) -> Self {
        let mut map: IndexMap<String, Label> = IndexMap::new();
        if t.is_empty() || labels.is_empty() {
            return Self { labels: map };
        }
        let n = t.len().min(labels.len());
        let mut cur_name = labels[0].clone();
        let mut cur_start = t[0];

        let insert_unique = |map: &mut IndexMap<String, Label>, base: &str, start: f64, stop: f64| {
            let mut key = base.to_string();
            let mut suffix = 1usize;
            while map.contains_key(&key) {
                key = format!("{}_{}", base, suffix);
                suffix += 1;
            }
            map.insert(key, Label::new(start, stop));
        };

        for i in 1..n {
            if labels[i] != cur_name {
                insert_unique(&mut map, &cur_name, cur_start, t[i]);
                cur_name = labels[i].clone();
                cur_start = t[i];
            }
        }
        let final_t = *t.last().unwrap();
        insert_unique(&mut map, &cur_name, cur_start, final_t);
        Self { labels: map }
    }

    pub fn to_array(&self, t: &[f64]) -> Vec<String> {
        let mut out = vec![String::new(); t.len()];
        let count = self.labels.len();
        for (idx, (name, lab)) in self.labels.iter().enumerate() {
            let inclusive = idx + 1 == count;
            for (ti, &tval) in t.iter().enumerate() {
                if lab.contains(tval, inclusive) {
                    out[ti] = name.clone();
                }
            }
        }
        out
    }

    pub fn offset(&self, dt: f64) -> Self {
        let mut map = IndexMap::new();
        for (k, v) in &self.labels {
            map.insert(k.clone(), v.offset(dt));
        }
        Self { labels: map }
    }

    pub fn slice(&self, tstart: f64, tstop: f64) -> Self {
        let mut map = IndexMap::new();
        for (k, v) in &self.labels {
            if v.intersects(tstart, tstop) {
                let s = v.start.max(tstart);
                let e = v.stop.min(tstop);
                map.insert(
                    k.clone(),
                    Label::with_sublabels(s, e, v.sublabels.slice(s, e)),
                );
            }
        }
        Self { labels: map }
    }

    pub fn to_iloc(&self, t: &[f64]) -> Result<Self, String> {
        let mut map = IndexMap::new();
        for (k, v) in &self.labels {
            map.insert(k.clone(), v.to_iloc(t)?);
        }
        Ok(Self { labels: map })
    }

    pub fn to_t(&self, t: &[f64]) -> Result<Self, String> {
        let mut map = IndexMap::new();
        for (k, v) in &self.labels {
            map.insert(k.clone(), v.to_t(t)?);
        }
        Ok(Self { labels: map })
    }

    /// Insert any missing keys from `expected_keys` at their sequence positions as zero-width labels.
    pub fn insert_list(&self, expected_keys: &[String]) -> Self {
        let mut new_map = IndexMap::new();
        let mut cur_pos = self
            .labels
            .values()
            .next()
            .map(|l| l.start)
            .unwrap_or(0.0);
        for key in expected_keys {
            if let Some(existing) = self.labels.get(key) {
                cur_pos = existing.stop;
                new_map.insert(key.clone(), existing.clone());
            } else {
                new_map.insert(key.clone(), Label::new(cur_pos, cur_pos));
            }
        }
        Self { labels: new_map }
    }

    /// Expand any labels in index space whose width is `< min_len` by borrowing from wider neighbors.
    pub fn expand(&self, min_len: f64) -> Self {
        if self.labels.len() <= 1 {
            return self.clone();
        }
        let keys: Vec<String> = self.labels.keys().cloned().collect();
        let mut starts: Vec<f64> = self.labels.values().map(|l| l.start).collect();
        let mut stops: Vec<f64> = self.labels.values().map(|l| l.stop).collect();

        for _iter in 0..100 {
            let mut changed = false;
            for i in 0..keys.len() {
                while stops[i] - starts[i] < min_len {
                    // Find nearest neighbor with width > min_len
                    let left_room = if i > 0 {
                        (stops[i - 1] - starts[i - 1]) - min_len
                    } else {
                        0.0
                    };
                    let right_room = if i + 1 < keys.len() {
                        (stops[i + 1] - starts[i + 1]) - min_len
                    } else {
                        0.0
                    };
                    if left_room <= 0.0 && right_room <= 0.0 {
                        break;
                    }
                    if left_room >= right_room {
                        starts[i] -= 1.0;
                        stops[i - 1] -= 1.0;
                    } else {
                        stops[i] += 1.0;
                        starts[i + 1] += 1.0;
                    }
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }

        let mut map = IndexMap::new();
        for (i, k) in keys.into_iter().enumerate() {
            let sub = self.labels[&k].sublabels.clone();
            map.insert(k, Label::with_sublabels(starts[i], stops[i], sub));
        }
        Self { labels: map }
    }

    /// Transfer labels from time array `a` to time array `b` along DTW `path` (`[(idx_a, idx_b)]`).
    pub fn transfer(&self, a: &[f64], b: &[f64], path: Option<&[[usize; 2]]>) -> Self {
        if a.is_empty() || b.is_empty() || self.is_empty() {
            return Self::new();
        }
        match path {
            None => {
                let a0 = a[0];
                let dur_a = (a.last().unwrap() - a0).max(1e-12);
                let b0 = b[0];
                let dur_b = b.last().unwrap() - b0;
                let mut map = IndexMap::new();
                for (k, v) in &self.labels {
                    let s = b0 + (v.start - a0) * dur_b / dur_a;
                    let e = b0 + (v.stop - a0) * dur_b / dur_a;
                    map.insert(k.clone(), Label::new(s, e));
                }
                Self { labels: map }
            }
            Some(dtw_path) => {
                let a_arr = self.to_array(a);
                let mut b_arr = vec![a_arr[0].clone(); b.len()];
                for &[ia, ib] in dtw_path {
                    if ia < a_arr.len() && ib < b_arr.len() {
                        b_arr[ib] = a_arr[ia].clone();
                    }
                }
                Self::read_array(b, &b_arr)
            }
        }
    }
}

/// Dictionary of named `LabelGroup`s (e.g., `"manoeuvre"`, `"element"`).
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct LabelGroups {
    pub groups: IndexMap<String, LabelGroup>,
}

impl LabelGroups {
    pub fn new() -> Self {
        Self {
            groups: IndexMap::new(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.groups.is_empty()
    }

    pub fn get(&self, group: &str) -> Option<&LabelGroup> {
        self.groups.get(group)
    }

    pub fn insert(&mut self, group: impl Into<String>, lg: LabelGroup) {
        self.groups.insert(group.into(), lg);
    }

    pub fn offset(&self, dt: f64) -> Self {
        let mut groups = IndexMap::new();
        for (k, v) in &self.groups {
            groups.insert(k.clone(), v.offset(dt));
        }
        Self { groups }
    }

    pub fn slice(&self, tstart: f64, tstop: f64) -> Self {
        let mut groups = IndexMap::new();
        for (k, v) in &self.groups {
            groups.insert(k.clone(), v.slice(tstart, tstop));
        }
        Self { groups }
    }
}
