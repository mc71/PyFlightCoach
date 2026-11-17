use crate::labels::LabelGroups;
use crate::state::State;
use pfc_geometry::Point;
use serde::{Deserialize, Serialize};

/// Result of Dynamic Time Warping (DTW) alignment between a `flown` and `template` `State`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Alignment {
    pub dist: f64,
    pub path: Vec<[usize; 2]>,
    pub aligned: State,
}

impl Alignment {
    /// Remove columns that are constant across the template, matching `Alignment.norm`.
    pub fn norm(tp: &[[f64; 3]], fl: &[[f64; 3]]) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
        if tp.is_empty() {
            return (Vec::new(), Vec::new());
        }
        let mut keep_cols = Vec::new();
        for col in 0..3 {
            let ref_val = tp[0][col];
            let all_close = tp.iter().all(|r| (r[col] - ref_val).abs() <= 1e-8);
            if !all_close {
                keep_cols.push(col);
            }
        }
        if keep_cols.is_empty() {
            keep_cols.extend([0, 1, 2]);
        }
        let tp_out = tp
            .iter()
            .map(|r| keep_cols.iter().map(|&c| r[c]).collect())
            .collect();
        let fl_out = fl
            .iter()
            .map(|r| keep_cols.iter().map(|&c| r[c]).collect())
            .collect();
        (tp_out, fl_out)
    }

    pub fn copy_labels(
        template: &State,
        flown: &State,
        path: Option<&[[usize; 2]]>,
        min_len: Option<f64>,
    ) -> State {
        let tp_t = template.t();
        let fl_t = flown.t();
        let mut new_groups = LabelGroups::new();

        for (group_name, tp_lg) in &template.labels.groups {
            let mut transferred = tp_lg.transfer(&tp_t, &fl_t, path);
            if let Some(ml) = min_len {
                if let Ok(iloc_lg) = transferred.to_iloc(&fl_t) {
                    let expected: Vec<String> = tp_lg.labels.keys().cloned().collect();
                    let expanded = iloc_lg.insert_list(&expected).expand(ml);
                    if let Ok(restored) = expanded.to_t(&fl_t) {
                        transferred = restored;
                    }
                }
            }
            new_groups.insert(group_name.clone(), transferred);
        }

        let mut out = flown.clone();
        out.labels = new_groups;
        out
    }

    /// Perform temporal DTW alignment between `flown` and `template` states based on weighted body angular rates (`rvel`).
    pub fn align(
        flown: &State,
        template: &State,
        window_radius: Option<usize>,
        mirror: bool,
        weights: Option<Point>,
        tp_weights: Option<Point>,
    ) -> Self {
        let weights = weights.unwrap_or_else(|| Point::new(1.0, 1.2, 0.5));
        let tp_weights = tp_weights.unwrap_or_else(|| Point::new(0.6, 0.6, 0.6));

        let get_brv = |brv: &Point| -> Point {
            let b = if mirror {
                Point::from_rows(
                    brv.data
                        .iter()
                        .map(|&[x, y, z]| [x.abs(), y, z.abs()])
                        .collect(),
                )
            } else {
                brv.clone()
            };
            &b * &weights
        };

        let fl = get_brv(&flown.rvel);
        let tp = get_brv(&(&template.rvel * &tp_weights));

        let (tp_norm, fl_norm) = Self::norm(&tp.data, &fl.data);
        let (dist, path) = dtw(&tp_norm, &fl_norm, window_radius);
        let aligned = Self::copy_labels(template, flown, Some(&path), Some(3.0));
        Self {
            dist,
            path,
            aligned,
        }
    }
}

fn euclidean(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| (x - y).powi(2))
        .sum::<f64>()
        .sqrt()
}

/// Dynamic Time Warping with optional Sakoe-Chiba band `window_radius`.
pub fn dtw(a: &[Vec<f64>], b: &[Vec<f64>], window_radius: Option<usize>) -> (f64, Vec<[usize; 2]>) {
    let n = a.len();
    let m = b.len();
    if n == 0 || m == 0 {
        return (0.0, Vec::new());
    }
    let w = window_radius
        .unwrap_or_else(|| n.max(m))
        .max(n.abs_diff(m));

    let cols = m + 1;
    let mut cost = vec![f64::INFINITY; (n + 1) * cols];
    cost[0] = 0.0;

    for i in 1..=n {
        let j_center = ((i as f64) * (m as f64) / (n as f64)).round() as usize;
        let j_start = j_center.saturating_sub(w).max(1);
        let j_end = (j_center + w).min(m);
        for j in j_start..=j_end {
            let d = euclidean(&a[i - 1], &b[j - 1]);
            let prev = cost[(i - 1) * cols + j]
                .min(cost[i * cols + (j - 1)])
                .min(cost[(i - 1) * cols + (j - 1)]);
            cost[i * cols + j] = d + prev;
        }
    }

    let total_dist = cost[n * cols + m];
    let mut path = Vec::with_capacity(n + m);
    let (mut i, mut j) = (n, m);
    while i > 0 && j > 0 {
        path.push([i - 1, j - 1]);
        if i == 1 && j == 1 {
            break;
        } else if i == 1 {
            j -= 1;
        } else if j == 1 {
            i -= 1;
        } else {
            let c_diag = cost[(i - 1) * cols + (j - 1)];
            let c_up = cost[(i - 1) * cols + j];
            let c_left = cost[i * cols + (j - 1)];
            if c_diag <= c_up && c_diag <= c_left {
                i -= 1;
                j -= 1;
            } else if c_up <= c_left {
                i -= 1;
            } else {
                j -= 1;
            }
        }
    }
    path.reverse();
    (total_dist, path)
}
