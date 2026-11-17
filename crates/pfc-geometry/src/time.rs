use crate::utils::{get_index, IndexDirection};
use serde::{Deserialize, Serialize};
use std::ops::{Add, Index};

/// Time and timestep sequence `[(t, dt); N]`, matching `geometry.Time`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Time {
    pub data: Vec<[f64; 2]>,
}

impl Time {
    pub fn new(t: f64, dt: f64) -> Self {
        Self {
            data: vec![[t, dt]],
        }
    }

    pub fn from_rows(data: Vec<[f64; 2]>) -> Self {
        Self { data }
    }

    pub fn from_scalar(t: f64) -> Self {
        Self::new(t, 1.0 / 25.0)
    }

    pub fn from_t(t: &[f64]) -> Self {
        if t.is_empty() {
            return Self { data: Vec::new() };
        }
        if t.len() == 1 {
            return Self::new(t[0], 1.0 / 25.0);
        }
        let mut data = Vec::with_capacity(t.len());
        for i in 0..t.len() - 1 {
            data.push([t[i], t[i + 1] - t[i]]);
        }
        let last_dt = data.last().unwrap()[1];
        data.push([*t.last().unwrap(), last_dt]);
        Self { data }
    }

    pub fn uniform(duration: f64, npoints: Option<usize>, minpoints: usize, freq: f64) -> Self {
        let n = npoints.unwrap_or_else(|| ((duration * freq).ceil() as usize).max(minpoints));
        if n <= 1 {
            return Self::from_scalar(0.0);
        }
        let ts: Vec<f64> = (0..n)
            .map(|i| duration * (i as f64) / ((n - 1) as f64))
            .collect();
        Self::from_t(&ts)
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

    pub fn slice(&self, start: usize, stop: usize) -> Self {
        Self {
            data: self.data[start..stop.min(self.data.len())].to_vec(),
        }
    }

    pub fn t(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[0]).collect()
    }

    pub fn dt(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[1]).collect()
    }

    #[inline]
    pub fn t0(&self) -> f64 {
        self.data[0][0]
    }

    #[inline]
    pub fn dt0(&self) -> f64 {
        self.data[0][1]
    }

    pub fn duration(&self) -> f64 {
        if self.data.is_empty() {
            0.0
        } else {
            self.data.last().unwrap()[0] - self.data[0][0]
        }
    }

    pub fn scale(&self, duration: f64) -> Self {
        let cur = self.duration();
        if cur.abs() < 1e-15 {
            return self.clone();
        }
        let fac = duration / cur;
        let t0 = self.t0();
        Self {
            data: self
                .data
                .iter()
                .map(|&[t, dt]| [(t - t0) * fac, dt * fac])
                .collect(),
        }
    }

    pub fn reset_zero(&self) -> Self {
        if self.data.is_empty() {
            return self.clone();
        }
        let t0 = self.t0();
        Self {
            data: self.data.iter().map(|&[t, dt]| [t - t0, dt]).collect(),
        }
    }

    pub fn extend(&self) -> Self {
        let mut data = self.data.clone();
        if let Some(&[last_t, last_dt]) = data.last() {
            data.push([last_t + last_dt, last_dt]);
        }
        Self { data }
    }

    pub fn interpolate_t(&self, t: f64) -> Result<f64, String> {
        get_index(&self.t(), t, IndexDirection::Forward, None)
    }

    pub fn linterp(&self, index: &[f64], target_ts: &[f64]) -> Self {
        let parent_t = self.t();
        let parent_dt = self.dt();
        let mut interp_t = Vec::with_capacity(target_ts.len());
        for &idx in target_ts {
            if index.is_empty() {
                interp_t.push(0.0);
            } else if idx <= index[0] || index.len() == 1 {
                interp_t.push(parent_t[0]);
            } else if idx >= *index.last().unwrap() {
                interp_t.push(*parent_t.last().unwrap());
            } else {
                let i = match index.binary_search_by(|v| v.partial_cmp(&idx).unwrap_or(std::cmp::Ordering::Equal)) {
                    Ok(k) => {
                        interp_t.push(parent_t[k]);
                        continue;
                    }
                    Err(k) => k.saturating_sub(1).min(index.len() - 2),
                };
                let denom = index[i + 1] - index[i];
                let frac = if denom.abs() < 1e-15 {
                    0.0
                } else {
                    (idx - index[i]) / denom
                };
                interp_t.push(parent_t[i] + frac * (parent_t[i + 1] - parent_t[i]));
            }
        }
        let mut new_time = Self::from_t(&interp_t);
        if let Some(&last_target) = target_ts.last() {
            // Match Python Time.linterp final dt adjustment
            if let Some(bfill_pos) = index.iter().position(|&v| v >= last_target) {
                let last_new_t = new_time.data.last().unwrap()[0];
                let last_dt = if (parent_t[bfill_pos] - last_new_t).abs() < 1e-12 {
                    parent_dt[bfill_pos]
                } else {
                    parent_t[bfill_pos] - last_new_t
                };
                if let Some(last_row) = new_time.data.last_mut() {
                    last_row[1] = last_dt;
                }
            }
        }
        new_time
    }
}

impl Index<usize> for Time {
    type Output = [f64; 2];
    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}

impl Add<f64> for &Time {
    type Output = Time;
    fn add(self, rhs: f64) -> Time {
        Time {
            data: self.data.iter().map(|&[t, dt]| [t + rhs, dt]).collect(),
        }
    }
}

impl Add<f64> for Time {
    type Output = Time;
    fn add(self, rhs: f64) -> Time {
        (&self) + rhs
    }
}
