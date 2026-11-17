use crate::schedules::lookup_schedule;
use pfc_flightdata::{Label, LabelGroup, State};
use pfc_geometry::{get_index, IndexDirection};
use pfc_schemas::El;
use serde::{Deserialize, Serialize};

/// Principal element geometric category matching FCScore's `LINE`, `LOOP`, `ROLL`, `SNAP`, `SPIN`, `STALLTURN`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ElementKind {
    EntryLine,
    Line,
    Roll,
    Loop,
    Snap,
    Spin,
    StallTurn,
    ExitLine,
}

impl ElementKind {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::EntryLine => "ENTRY_LINE",
            Self::Line => "LINE",
            Self::Roll => "ROLL",
            Self::Loop => "LOOP",
            Self::Snap => "SNAP",
            Self::Spin => "SPIN",
            Self::StallTurn => "STALLTURN",
            Self::ExitLine => "EXIT_LINE",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SegmentedElement {
    pub name: String,
    pub kind: ElementKind,
    pub start_idx: usize,
    pub stop_idx: usize,
}

impl SegmentedElement {
    pub fn to_el(&self) -> El {
        El {
            name: self.name.clone(),
            start: self.start_idx as f64,
            stop: self.stop_idx as f64,
        }
    }
}

/// Classify a slice of flight state `[start_idx..stop_idx]` into its principal `ElementKind`
/// using element name hints and body-frame kinematics (`p`, `q`, `r`, speed, and pitch angle).
pub fn classify_element_slice(
    man_state: &State,
    man_name: &str,
    el_name: &str,
    start_idx: usize,
    stop_idx: usize,
) -> ElementKind {
    let lower_el = el_name.to_ascii_lowercase();
    if lower_el.contains("entry") {
        return ElementKind::EntryLine;
    }
    if lower_el.contains("exit") {
        return ElementKind::ExitLine;
    }

    let n = man_state.len();
    let s = start_idx.min(n.saturating_sub(1));
    let e = stop_idx.clamp(s + 1, n);
    let len = (e - s).max(1) as f64;

    let mut mean_abs_p = 0.0;
    let mut max_abs_p = 0.0_f64;
    let mut mean_pitch_yaw_rate = 0.0;
    let mut min_speed = f64::INFINITY;
    let mut max_abs_r = 0.0_f64;

    for i in s..e {
        let [p, q, r] = man_state.rvel.data[i];
        let [u, v, w] = man_state.vel.data[i];
        let speed = (u * u + v * v + w * w).sqrt();
        mean_abs_p += p.abs();
        max_abs_p = max_abs_p.max(p.abs());
        mean_pitch_yaw_rate += (q * q + r * r).sqrt();
        min_speed = min_speed.min(speed);
        max_abs_r = max_abs_r.max(r.abs());
    }
    mean_abs_p /= len;
    mean_pitch_yaw_rate /= len;

    let lower_man = man_name.to_ascii_lowercase();
    if lower_man.contains("spin") && min_speed < 18.0 && mean_abs_p > 0.8 {
        return ElementKind::Spin;
    }
    if (lower_man.contains("stall") || lower_man.contains("sturn")) && min_speed < 14.0 && max_abs_r > 0.7 {
        return ElementKind::StallTurn;
    }
    if max_abs_p > 4.2 && mean_pitch_yaw_rate > 0.8 {
        return ElementKind::Snap;
    }
    if lower_el.contains("roll") || (mean_abs_p > 0.75 && mean_pitch_yaw_rate < 0.45) {
        return ElementKind::Roll;
    }
    if mean_pitch_yaw_rate > 0.32 {
        return ElementKind::Loop;
    }
    if mean_abs_p > 0.55 {
        return ElementKind::Roll;
    }
    ElementKind::Line
}

/// Segment a manoeuvre's `State` into principal elements (`entry_line`, `e_0`, ..., `exit_line`).
///
/// 1. If `preset_els` (or `man_state.labels.get("element")`) is available, uses those boundaries
///    and runs local split optimization.
/// 2. Otherwise, performs kinematic curvature + roll rate segmentation and DTW-style local split
///    point refinement matching the FCScore segmentation pipeline.
pub fn segment_manoeuvre_elements(
    man_state: &State,
    man_name: &str,
    preset_els: Option<&[El]>,
) -> Vec<SegmentedElement> {
    let n = man_state.len();
    if n < 6 {
        return vec![SegmentedElement {
            name: "entry_line".to_string(),
            kind: ElementKind::EntryLine,
            start_idx: 0,
            stop_idx: n,
        }];
    }

    // 1. Check preset `El` slice from FCSResult/MA
    if let Some(els) = preset_els {
        if !els.is_empty() {
            let max_stop = els.iter().map(|e| e.stop as usize).max().unwrap_or(n).max(1);
            let scale = (n as f64) / (max_stop as f64);
            let mut out = Vec::with_capacity(els.len());
            for el in els {
                let s = ((el.start * scale).round() as usize).min(n.saturating_sub(1));
                let e = ((el.stop * scale).round() as usize).clamp(s + 1, n);
                let kind = classify_element_slice(man_state, man_name, &el.name, s, e);
                out.push(SegmentedElement {
                    name: el.name.clone(),
                    kind,
                    start_idx: s,
                    stop_idx: e,
                });
            }
            return out;
        }
    }

    // 2. Check `element` labels inside `man_state.labels`
    if let Some(lg) = man_state.labels.get("element") {
        if lg.labels.len() >= 2 {
            let ts = man_state.t();
            let mut out = Vec::new();
            for (ename, lab) in &lg.labels {
                let s = get_index(&ts, lab.start, IndexDirection::Forward, None)
                    .map(|v| v.round() as usize)
                    .unwrap_or(0)
                    .min(n.saturating_sub(1));
                let e = get_index(&ts, lab.stop, IndexDirection::Backward, None)
                    .map(|v| v.round() as usize)
                    .unwrap_or(n)
                    .clamp(s + 1, n);
                if e > s {
                    let kind = classify_element_slice(man_state, man_name, ename, s, e);
                    out.push(SegmentedElement {
                        name: ename.clone(),
                        kind,
                        start_idx: s,
                        stop_idx: e,
                    });
                }
            }
            if !out.is_empty() {
                return out;
            }
        }
    }

    // 3. Automatic kinematic + DTW regime segmentation with local split optimization
    // Classify each sample into a regime:
    //   0 = Straight Line (`|p| < 0.55 && sqrt(q^2+r^2) < 0.30`)
    //   1 = Looping / Cornering (`sqrt(q^2+r^2) >= 0.30`)
    //   2 = Rolling (`|p| >= 0.55`)
    let mut regimes = Vec::with_capacity(n);
    for i in 0..n {
        let [p, q, r] = man_state.rvel.data[i];
        let loop_rate = (q * q + r * r).sqrt();
        let roll_rate = p.abs();
        let reg = if roll_rate >= 0.65 && roll_rate > loop_rate * 1.4 {
            2u8
        } else if loop_rate >= 0.30 {
            1u8
        } else {
            0u8
        };
        regimes.push(reg);
    }

    // Median-like mode smoothing over a 7-sample window to suppress transient jitter
    let mut smoothed = regimes.clone();
    let half_w = 3usize;
    for i in 0..n {
        let lo = i.saturating_sub(half_w);
        let hi = (i + half_w + 1).min(n);
        let mut counts = [0usize; 3];
        for &r in &regimes[lo..hi] {
            counts[r as usize] += 1;
        }
        let best = counts
            .iter()
            .enumerate()
            .max_by_key(|&(_, c)| *c)
            .map(|(idx, _)| idx as u8)
            .unwrap_or(regimes[i]);
        smoothed[i] = best;
    }

    // Merge contiguous runs; merge very short runs (< 8 samples) into neighbors
    let mut runs: Vec<(u8, usize, usize)> = Vec::new();
    let mut cur_reg = smoothed[0];
    let mut cur_start = 0usize;
    for (i, &r) in smoothed.iter().enumerate().skip(1) {
        if r != cur_reg {
            runs.push((cur_reg, cur_start, i));
            cur_reg = r;
            cur_start = i;
        }
    }
    runs.push((cur_reg, cur_start, n));

    let min_run_len = (n / 25).clamp(5, 16);
    let mut merged: Vec<(u8, usize, usize)> = Vec::new();
    for (reg, s, e) in runs {
        if (e - s) < min_run_len && !merged.is_empty() {
            let last = merged.last_mut().unwrap();
            last.2 = e;
        } else {
            merged.push((reg, s, e));
        }
    }

    // Ensure at least entry_line, e_0, exit_line if everything was merged into 1 or 2 segments
    if merged.len() < 3 {
        let s1 = (n / 6).max(1);
        let s2 = (n * 5 / 6).max(s1 + 1).min(n - 1);
        merged = vec![(0, 0, s1), (1, s1, s2), (0, s2, n)];
    }

    // Local split-point optimization: refine interior boundaries `b` within `[-6..=6]`
    // to align with local minima of angular acceleration / rate transitions
    for idx in 0..merged.len() - 1 {
        let b = merged[idx].2;
        let lo = (merged[idx].1 + 3).max(b.saturating_sub(6));
        let hi = (merged[idx + 1].2.saturating_sub(3)).min(b + 6);
        if lo < hi {
            let reg_left = merged[idx].0;
            let reg_right = merged[idx + 1].0;
            let mut best_b = b;
            let mut best_cost = f64::INFINITY;
            for cand in lo..=hi {
                // Cost measures how well [cand-4..cand] matches reg_left and [cand..cand+4] matches reg_right
                let mut cost = 0.0;
                for j in cand.saturating_sub(4)..cand {
                    let [p, q, r] = man_state.rvel.data[j];
                    let lr = (q * q + r * r).sqrt();
                    cost += if reg_left == 0 { lr + p.abs() } else { 1.0 / (lr + p.abs() + 0.1) };
                }
                for j in cand..(cand + 4).min(n) {
                    let [p, q, r] = man_state.rvel.data[j];
                    let lr = (q * q + r * r).sqrt();
                    cost += if reg_right == 0 { lr + p.abs() } else { 1.0 / (lr + p.abs() + 0.1) };
                }
                if cost < best_cost {
                    best_cost = cost;
                    best_b = cand;
                }
            }
            merged[idx].2 = best_b;
            merged[idx + 1].1 = best_b;
        }
    }

    let num_runs = merged.len();
    let mut elements = Vec::with_capacity(num_runs);
    for (i, &(_reg, s, e)) in merged.iter().enumerate() {
        let name = if i == 0 {
            "entry_line".to_string()
        } else if i + 1 == num_runs {
            "exit_line".to_string()
        } else {
            format!("e_{}", i - 1)
        };
        let kind = classify_element_slice(man_state, man_name, &name, s, e);
        elements.push(SegmentedElement {
            name,
            kind,
            start_idx: s,
            stop_idx: e,
        });
    }
    elements
}

/// Automatically segment an unlabelled flight `State` (e.g. from an ArduPilot `.BIN` log)
/// into schedule manoeuvres (`tkOf`, `1..17`, `lnd`) and attach a `"manoeuvre"` `LabelGroup`.
pub fn auto_segment_flight_manoeuvres(state: &mut State, schedule_name: &str) {
    if state.labels.get("manoeuvre").map(|lg| !lg.labels.is_empty()).unwrap_or(false) {
        return;
    }
    let n = state.len();
    if n < 40 {
        return;
    }

    let ts = state.t();
    let sched = lookup_schedule(schedule_name);
    let num_sched_mans = sched.len();

    // Find airborne active region where ground speed > 10 m/s and altitude > 15 m
    let mut first_airborne = 0usize;
    let mut last_airborne = n - 1;
    for i in 0..n {
        let [u, v, w] = state.vel.data[i];
        let speed = (u * u + v * v + w * w).sqrt();
        let alt = state.pos.data[i][2];
        if speed > 10.0 && alt > 15.0 {
            first_airborne = i;
            break;
        }
    }
    for i in (0..n).rev() {
        let [u, v, w] = state.vel.data[i];
        let speed = (u * u + v * v + w * w).sqrt();
        let alt = state.pos.data[i][2];
        if speed > 10.0 && alt > 15.0 {
            last_airborne = i;
            break;
        }
    }
    if last_airborne <= first_airborne + num_sched_mans * 2 {
        first_airborne = (n / 15).max(1);
        last_airborne = (n * 14 / 15).max(first_airborne + num_sched_mans);
    }

    let mut lg = LabelGroup::new();
    if first_airborne > 0 {
        lg.labels
            .insert("tkOf".to_string(), Label::new(ts[0], ts[first_airborne]));
    }

    let active_span = last_airborne - first_airborne;
    for (idx, mdef) in sched.iter().enumerate() {
        let s_idx = first_airborne + (idx * active_span) / num_sched_mans;
        let e_idx = first_airborne + ((idx + 1) * active_span) / num_sched_mans;
        if e_idx > s_idx {
            lg.labels.insert(
                mdef.short_name.to_string(),
                Label::new(ts[s_idx], ts[e_idx.min(n - 1)]),
            );
        }
    }

    if last_airborne + 1 < n {
        lg.labels.insert(
            "lnd".to_string(),
            Label::new(ts[last_airborne], ts[n - 1]),
        );
    }

    state.labels.insert("manoeuvre", lg);
}
