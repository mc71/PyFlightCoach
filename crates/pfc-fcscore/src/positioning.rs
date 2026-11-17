use crate::intra::scale_downgrade;
use pfc_flightdata::State;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ManoeuvrePositioningReport {
    pub center_offset_deg: f64,
    pub max_lateral_angle_deg: f64,
    pub max_vertical_angle_deg: f64,
    pub mean_depth_y_m: f64,
    pub centering_downgrade: f64,
    pub box_boundary_downgrade: f64,
    pub depth_downgrade: f64,
    pub raw_positioning_total: f64,
}

impl ManoeuvrePositioningReport {
    pub fn scaled_total(&self, difficulty: i32, truncate: bool) -> f64 {
        scale_downgrade(self.centering_downgrade, difficulty, truncate)
            + scale_downgrade(self.box_boundary_downgrade, difficulty, truncate)
            + scale_downgrade(self.depth_downgrade, difficulty, truncate)
    }
}

/// Evaluate F3A aerobatic box positioning & centering downgrades for a manoeuvre `State`.
///
/// - **Centering**: For centre manoeuvres (`is_centered = true`), checks the angular offset
///   of the principal manoeuvre midpoint `(X_mid, Y_mid)` from the centre line `X = 0`.
/// - **Box Infringement**: Checks lateral `|atan2(X, Y)| > 60°` and vertical `atan2(Z, Y) > 60°`.
/// - **Depth**: Checks distance `Y` from the pilot/judges line (`150m..175m` ideal; downgrade
///   scales past `185m` up to `250m`).
pub fn evaluate_manoeuvre_positioning(
    man_state: &State,
    is_centered: bool,
    entry_stop_idx: usize,
    exit_start_idx: usize,
) -> ManoeuvrePositioningReport {
    let n = man_state.len();
    if n == 0 {
        return ManoeuvrePositioningReport {
            center_offset_deg: 0.0,
            max_lateral_angle_deg: 0.0,
            max_vertical_angle_deg: 0.0,
            mean_depth_y_m: 170.0,
            centering_downgrade: 0.0,
            box_boundary_downgrade: 0.0,
            depth_downgrade: 0.0,
            raw_positioning_total: 0.0,
        };
    }

    let s = entry_stop_idx.min(n - 1);
    let e = exit_start_idx.clamp(s + 1, n);

    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut sum_y = 0.0;
    let mut max_lat_deg = 0.0_f64;
    let mut max_vert_deg = 0.0_f64;

    for i in s..e {
        let [x, y, z] = man_state.pos.data[i];
        let depth = y.abs().max(20.0);
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        sum_y += depth;

        let lat_deg = (x.abs() / depth).atan().to_degrees();
        let vert_deg = (z.max(0.0) / depth).atan().to_degrees();
        max_lat_deg = max_lat_deg.max(lat_deg);
        max_vert_deg = max_vert_deg.max(vert_deg);
    }

    let mean_depth_y_m = sum_y / ((e - s) as f64);
    let mid_x = 0.5 * (min_x + max_x);
    let center_offset_deg = (mid_x.abs() / mean_depth_y_m.max(50.0)).atan().to_degrees();

    let centering_downgrade = if is_centered {
        // 1 pt per 10 deg off-center beyond 1.5 deg tolerance
        ((center_offset_deg - 1.5).max(0.0) / 10.0).clamp(0.0, 3.0)
    } else {
        0.0
    };

    let lat_excursion = (max_lat_deg - 60.0).max(0.0);
    let vert_excursion = (max_vert_deg - 60.0).max(0.0);
    let box_boundary_downgrade = ((lat_excursion + vert_excursion) / 5.0).clamp(0.0, 3.0);

    let depth_downgrade = if mean_depth_y_m > 180.0 {
        ((mean_depth_y_m - 180.0) / 55.0).clamp(0.0, 2.0)
    } else if mean_depth_y_m < 90.0 {
        ((90.0 - mean_depth_y_m) / 40.0).clamp(0.0, 2.0)
    } else {
        0.0
    };

    let raw_positioning_total = centering_downgrade + box_boundary_downgrade + depth_downgrade;

    ManoeuvrePositioningReport {
        center_offset_deg,
        max_lateral_angle_deg: max_lat_deg,
        max_vertical_angle_deg: max_vert_deg,
        mean_depth_y_m,
        centering_downgrade,
        box_boundary_downgrade,
        depth_downgrade,
        raw_positioning_total,
    }
}
