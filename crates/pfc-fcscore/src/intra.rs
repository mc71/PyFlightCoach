use crate::segmentation::{ElementKind, SegmentedElement};
use pfc_flightdata::State;
use pfc_geometry::butterworth_lowpass_filtfilt;
use serde::Serialize;
use std::f64::consts::PI;

/// Individual intra-element downgrade measurement & result matching FCScore's 6-step pipeline.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct IntraDowngradeItem {
    pub name: &'static str,
    pub measure_name: &'static str,
    pub criteria: &'static str,
    pub peak_error_deg_or_pct: f64,
    pub mean_visibility: f64,
    pub raw_downgrade: f64,
}

/// Complete intra-element assessment for a single segmented element.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ElementIntraReport {
    pub element_name: String,
    pub kind: ElementKind,
    pub start_idx: usize,
    pub stop_idx: usize,
    pub duration_s: f64,
    pub mean_speed_mps: f64,
    pub mean_radius_m: Option<f64>,
    pub mean_roll_rate_dps: f64,
    pub line_length_m: f64,
    pub items: Vec<IntraDowngradeItem>,
    pub raw_intra_total: f64,
}

/// Apply FCScore difficulty curve (`1 = Easy`, `2 = Medium`, `3 = Hard`) and optional `0.5` truncation
/// to an individual downgrade value `d` (F3A max single downgrade reference = `6.0`).
#[inline]
pub fn scale_downgrade(d: f64, difficulty: i32, truncate: bool) -> f64 {
    if d <= 1e-9 {
        return 0.0;
    }
    let d_max = 6.0_f64;
    let gamma = match difficulty {
        1 => 1.55,
        2 => 1.24,
        _ => 1.0,
    };
    let scaled = if d >= d_max {
        d
    } else {
        d_max * (d / d_max).powf(gamma)
    };
    if truncate {
        (scaled * 2.0).floor() * 0.5
    } else {
        scaled
    }
}

/// Judge's line-of-sight unit vector from pilot origin `(0,0,0)` to aircraft position `pos`.
#[inline]
fn los_unit(pos: [f64; 3]) -> [f64; 3] {
    let n = (pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]).sqrt();
    if n <= 1e-9 {
        [0.0, 1.0, 0.0]
    } else {
        [pos[0] / n, pos[1] / n, pos[2] / n]
    }
}

/// Visibility factor `[0.25..=1.0]` for track errors along direction `err_dir` given judge LOS.
#[inline]
fn visibility_vector_direction(pos: [f64; 3], err_dir: [f64; 3]) -> f64 {
    let los = los_unit(pos);
    let dot = (los[0] * err_dir[0] + los[1] * err_dir[1] + los[2] * err_dir[2]).clamp(-1.0, 1.0);
    (1.0 - dot * dot).max(0.0).sqrt().clamp(0.25, 1.0)
}

/// Visibility factor `[0.35..=1.0]` for roll angle errors given body Z axis and judge LOS.
#[inline]
fn visibility_roll_angle(pos: [f64; 3], body_z: [f64; 3]) -> f64 {
    let los = los_unit(pos);
    let dot = (los[0] * body_z[0] + los[1] * body_z[1] + los[2] * body_z[2]).clamp(-1.0, 1.0);
    (1.0 - 0.65 * dot * dot).max(0.15).sqrt().clamp(0.35, 1.0)
}

/// Nearest F3A cardinal bank angle (`0, ±90°, ±180°` or `±45°, ±135°`) error in radians.
#[inline]
fn cardinal_roll_error_rad(roll_rad: f64) -> f64 {
    let step = PI / 2.0; // 90 deg cardinal wings-level / knife-edge / inverted
    let nearest = (roll_rad / step).round() * step;
    (roll_rad - nearest).abs()
}

/// Evaluate the 6-step FCScore Intra-Element Assessment for one segmented element.
pub fn evaluate_element_intra(
    man_state: &State,
    el: &SegmentedElement,
    prev_kind: Option<ElementKind>,
    next_kind: Option<ElementKind>,
) -> ElementIntraReport {
    let n_total = man_state.len();
    let s = el.start_idx.min(n_total.saturating_sub(1));
    let e = el.stop_idx.clamp(s + 1, n_total);
    let count = e - s;

    let ts = man_state.t();
    let duration_s = (ts[e - 1] - ts[s]).max(1e-3);
    let eul = man_state.att.to_euler();
    let rot_mats = man_state.att.to_rotation_matrix();

    // Compute element kinematic summaries
    let mut speeds = Vec::with_capacity(count);
    let mut roll_errs_vis = Vec::with_capacity(count);
    let mut track_y_errs_vis = Vec::with_capacity(count);
    let mut track_z_errs_vis = Vec::with_capacity(count);
    let mut roll_rates_dps = Vec::with_capacity(count);
    let mut loop_radii = Vec::new();
    let mut vis_accum = 0.0;

    // World velocity vectors for track measurement
    let wvel = man_state.att.transform_point(&man_state.vel);

    // Reference pitch angle of the element (snapped to nearest 45 deg for lines)
    let mut pitch_tracks = Vec::with_capacity(count);
    for i in s..e {
        let [vx, vy, vz] = wvel.data[i];
        let horiz = (vx * vx + vy * vy).sqrt().max(1e-6);
        pitch_tracks.push(vz.atan2(horiz));
    }
    let mean_pitch_track = pitch_tracks.iter().sum::<f64>() / (count as f64);
    let target_pitch_track = (mean_pitch_track / (PI / 4.0)).round() * (PI / 4.0);

    for i in s..e {
        let pos = man_state.pos.data[i];
        let [u, v, w] = man_state.vel.data[i];
        let speed = (u * u + v * v + w * w).sqrt().max(1.0);
        speeds.push(speed);

        let [roll, _pitch, _yaw] = eul.data[i];
        let [p, q, r] = man_state.rvel.data[i];
        roll_rates_dps.push(p.to_degrees());

        let rmat = rot_mats[i];
        let body_z = [rmat[0][2], rmat[1][2], rmat[2][2]];
        let v_roll = visibility_roll_angle(pos, body_z);
        let v_y = visibility_vector_direction(pos, [0.0, 1.0, 0.0]);
        let v_z = visibility_vector_direction(pos, [0.0, 0.0, 1.0]);
        vis_accum += (v_roll + v_y + v_z) / 3.0;

        // 1. Roll angle error (weighted by visibility)
        let r_err_deg = cardinal_roll_error_rad(roll).to_degrees() * v_roll;
        roll_errs_vis.push(r_err_deg);

        // 2. Cross-box depth track error `rf_y_track` (deviation from X-Z judging plane)
        let [_vx, vy, _vz] = wvel.data[i];
        let track_y_deg = (vy / speed).clamp(-1.0, 1.0).asin().to_degrees().abs() * v_y;
        track_y_errs_vis.push(track_y_deg);

        // 3. Vertical inclination track error `rf_z_track`
        let track_z_deg = (pitch_tracks[i - s] - target_pitch_track)
            .to_degrees()
            .abs()
            * v_z;
        track_z_errs_vis.push(track_z_deg);

        // 4. Instantaneous loop radius R = speed / sqrt(q^2 + r^2)
        let omega_perp = (q * q + r * r).sqrt();
        if omega_perp > 0.18 {
            loop_radii.push((speed / omega_perp).clamp(8.0, 450.0));
        }
    }

    // Step 3: Smooth measurement signals using Butterworth filtfilt
    let roll_smoothed = if count >= 8 {
        butterworth_lowpass_filtfilt(&roll_errs_vis, 2.5, 25.0, 2)
    } else {
        roll_errs_vis.clone()
    };
    let track_y_smoothed = if count >= 8 {
        butterworth_lowpass_filtfilt(&track_y_errs_vis, 2.0, 25.0, 2)
    } else {
        track_y_errs_vis.clone()
    };
    let track_z_smoothed = if count >= 8 {
        butterworth_lowpass_filtfilt(&track_z_errs_vis, 2.0, 25.0, 2)
    } else {
        track_z_errs_vis.clone()
    };

    let mean_speed_mps = speeds.iter().sum::<f64>() / (count as f64);
    let mean_roll_rate_dps =
        roll_rates_dps.iter().map(|x| x.abs()).sum::<f64>() / (count as f64);
    let p_start = man_state.pos.data[s];
    let p_end = man_state.pos.data[e - 1];
    let line_length_m = ((p_end[0] - p_start[0]).powi(2)
        + (p_end[1] - p_start[1]).powi(2)
        + (p_end[2] - p_start[2]).powi(2))
    .sqrt();
    let mean_vis = (vis_accum / (count as f64)).clamp(0.3, 1.0);

    // Step 4: Selector — relax pre-spin / post-stallturn boundaries just like FCScore
    let relax_factor = if matches!(next_kind, Some(ElementKind::Spin))
        || matches!(prev_kind, Some(ElementKind::Spin | ElementKind::StallTurn))
    {
        0.55
    } else {
        1.0
    };

    let mut items = Vec::new();
    let mut mean_radius_m = None;

    match el.kind {
        ElementKind::EntryLine | ElementKind::ExitLine | ElementKind::Line => {
            // Roll angle continuous error (F3A: ~1 pt per 15 deg, scaled for continuous line hold)
            let peak_roll = roll_smoothed
                .iter()
                .copied()
                .fold(0.0_f64, f64::max)
                * relax_factor;
            let roll_dg = ((peak_roll - 1.5).max(0.0) / 18.0).clamp(0.0, 2.5);
            items.push(IntraDowngradeItem {
                name: "roll_angle",
                measure_name: "roll_angle",
                criteria: "Continuous",
                peak_error_deg_or_pct: peak_roll,
                mean_visibility: mean_vis,
                raw_downgrade: roll_dg,
            });

            // Cross-box track_y error
            let peak_ty = track_y_smoothed
                .iter()
                .copied()
                .fold(0.0_f64, f64::max)
                * relax_factor;
            let ty_dg = ((peak_ty - 2.0).max(0.0) / 22.0).clamp(0.0, 2.0);
            items.push(IntraDowngradeItem {
                name: "track_y",
                measure_name: "rf_y_track",
                criteria: "Continuous",
                peak_error_deg_or_pct: peak_ty,
                mean_visibility: mean_vis,
                raw_downgrade: ty_dg,
            });

            // Pitch inclination track_z error
            let peak_tz = track_z_smoothed
                .iter()
                .copied()
                .fold(0.0_f64, f64::max)
                * relax_factor;
            let tz_dg = ((peak_tz - 2.0).max(0.0) / 22.0).clamp(0.0, 2.0);
            items.push(IntraDowngradeItem {
                name: "track_z",
                measure_name: "rf_z_track",
                criteria: "Continuous",
                peak_error_deg_or_pct: peak_tz,
                mean_visibility: mean_vis,
                raw_downgrade: tz_dg,
            });
        }
        ElementKind::Roll => {
            // Roll rate consistency (Deviation criteria) + end bank angle (Single criteria)
            let abs_rates: Vec<f64> = roll_rates_dps.iter().map(|r| r.abs()).collect();
            let mean_r = abs_rates.iter().sum::<f64>() / (abs_rates.len() as f64).max(1.0);
            let std_r = (abs_rates
                .iter()
                .map(|r| (r - mean_r).powi(2))
                .sum::<f64>()
                / (abs_rates.len() as f64).max(1.0))
            .sqrt();
            let rel_dev = if mean_r > 15.0 { std_r / mean_r } else { 0.0 };
            let rate_dg = ((rel_dev - 0.18).max(0.0) * 0.85).clamp(0.0, 2.0);
            items.push(IntraDowngradeItem {
                name: "roll_rate",
                measure_name: "roll_rate",
                criteria: "Deviation",
                peak_error_deg_or_pct: rel_dev * 100.0,
                mean_visibility: mean_vis,
                raw_downgrade: rate_dg,
            });

            let last_roll_err = *roll_smoothed.last().unwrap_or(&0.0);
            let end_roll_dg = ((last_roll_err - 1.5).max(0.0) / 15.0).clamp(0.0, 2.5);
            items.push(IntraDowngradeItem {
                name: "roll_angle",
                measure_name: "roll_angle",
                criteria: "Single (last)",
                peak_error_deg_or_pct: last_roll_err,
                mean_visibility: mean_vis,
                raw_downgrade: end_roll_dg,
            });
        }
        ElementKind::Loop => {
            // Radius consistency (`radius_curve_fit` / `curvature`), `axial_track`, and `roll_angle`
            if !loop_radii.is_empty() {
                let r_mean = loop_radii.iter().sum::<f64>() / (loop_radii.len() as f64);
                mean_radius_m = Some(r_mean);
                let r_std = (loop_radii
                    .iter()
                    .map(|r| (r - r_mean).powi(2))
                    .sum::<f64>()
                    / (loop_radii.len() as f64))
                .sqrt();
                let cv = (r_std / r_mean.max(10.0)).clamp(0.0, 1.5);
                let radius_dg = ((cv - 0.12).max(0.0) * 1.15).clamp(0.0, 2.5);
                items.push(IntraDowngradeItem {
                    name: "radius",
                    measure_name: "radius_curve_fit",
                    criteria: "Deviation",
                    peak_error_deg_or_pct: cv * 100.0,
                    mean_visibility: mean_vis,
                    raw_downgrade: radius_dg,
                });
            }

            let axial_err = track_y_smoothed.iter().copied().fold(0.0_f64, f64::max);
            let axial_dg = ((axial_err - 2.5).max(0.0) / 24.0).clamp(0.0, 2.0);
            items.push(IntraDowngradeItem {
                name: "axial_track",
                measure_name: "loop_axial_track",
                criteria: "Continuous",
                peak_error_deg_or_pct: axial_err,
                mean_visibility: mean_vis,
                raw_downgrade: axial_dg,
            });

            let loop_roll_err = roll_smoothed.iter().copied().fold(0.0_f64, f64::max);
            let loop_roll_dg = ((loop_roll_err - 2.0).max(0.0) / 20.0).clamp(0.0, 2.0);
            items.push(IntraDowngradeItem {
                name: "roll_angle",
                measure_name: "roll_angle_p",
                criteria: "Continuous",
                peak_error_deg_or_pct: loop_roll_err,
                mean_visibility: mean_vis,
                raw_downgrade: loop_roll_dg,
            });
        }
        ElementKind::Snap | ElementKind::Spin => {
            let last_roll = *roll_smoothed.last().unwrap_or(&0.0);
            let turns_dg = ((last_roll - 2.0).max(0.0) / 14.0).clamp(0.0, 2.5);
            items.push(IntraDowngradeItem {
                name: "turns",
                measure_name: "roll_angle_y",
                criteria: "Single (last)",
                peak_error_deg_or_pct: last_roll,
                mean_visibility: mean_vis,
                raw_downgrade: turns_dg,
            });

            let break_dg = if mean_roll_rate_dps < 95.0 {
                ((95.0 - mean_roll_rate_dps) / 60.0).clamp(0.0, 1.5)
            } else {
                0.0
            };
            items.push(IntraDowngradeItem {
                name: "autorotation",
                measure_name: "autorotation_rate",
                criteria: "Bounded",
                peak_error_deg_or_pct: mean_roll_rate_dps,
                mean_visibility: mean_vis,
                raw_downgrade: break_dg,
            });
        }
        ElementKind::StallTurn => {
            // Stallturn width (in wingspans, 2m wingspan reference) & wing levelness
            let xs: Vec<f64> = (s..e).map(|i| man_state.pos.data[i][0]).collect();
            let min_x = xs.iter().copied().fold(f64::INFINITY, f64::min);
            let max_x = xs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            let width_spans = (max_x - min_x) / 2.0;
            let width_dg = ((width_spans - 1.5).max(0.0) * 0.35).clamp(0.0, 2.5);
            items.push(IntraDowngradeItem {
                name: "width",
                measure_name: "stallturn_width",
                criteria: "Peak",
                peak_error_deg_or_pct: width_spans,
                mean_visibility: mean_vis,
                raw_downgrade: width_dg,
            });

            let z_roll = roll_smoothed.iter().copied().fold(0.0_f64, f64::max);
            let z_roll_dg = ((z_roll - 2.0).max(0.0) / 18.0).clamp(0.0, 2.0);
            items.push(IntraDowngradeItem {
                name: "roll_angle",
                measure_name: "roll_angle_z",
                criteria: "Continuous",
                peak_error_deg_or_pct: z_roll,
                mean_visibility: mean_vis,
                raw_downgrade: z_roll_dg,
            });
        }
    }

    let raw_intra_total: f64 = items.iter().map(|it| it.raw_downgrade).sum();

    ElementIntraReport {
        element_name: el.name.clone(),
        kind: el.kind,
        start_idx: s,
        stop_idx: e,
        duration_s,
        mean_speed_mps,
        mean_radius_m,
        mean_roll_rate_dps,
        line_length_m,
        items,
        raw_intra_total,
    }
}
