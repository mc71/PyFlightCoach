use crate::intra::{scale_downgrade, ElementIntraReport};
use crate::segmentation::ElementKind;
use serde::Serialize;

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct InterDowngradeItem {
    pub name: &'static str,
    pub description: String,
    pub relative_error_pct: f64,
    pub raw_downgrade: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ManoeuvreInterReport {
    pub items: Vec<InterDowngradeItem>,
    pub raw_inter_total: f64,
}

impl ManoeuvreInterReport {
    pub fn scaled_total(&self, difficulty: i32, truncate: bool) -> f64 {
        self.items
            .iter()
            .map(|it| scale_downgrade(it.raw_downgrade, difficulty, truncate))
            .sum()
    }
}

/// Evaluate inter-element downgrades across the principal elements of a manoeuvre:
/// 1. Loop radius matching across multiple `Loop` elements.
/// 2. Interior `Line` length symmetry around rolls/corners.
/// 3. `Roll` rate consistency across multiple roll elements.
/// 4. Entry vs Exit altitude/speed consistency.
pub fn evaluate_manoeuvre_inter(elements: &[ElementIntraReport]) -> ManoeuvreInterReport {
    let mut items = Vec::new();

    // 1. Loop radius consistency across Loop elements
    let radii: Vec<f64> = elements
        .iter()
        .filter_map(|e| e.mean_radius_m)
        .collect();
    if radii.len() >= 2 {
        let mean_r = radii.iter().sum::<f64>() / (radii.len() as f64);
        let max_r = radii.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let min_r = radii.iter().copied().fold(f64::INFINITY, f64::min);
        let rel_diff = (max_r - min_r) / mean_r.max(10.0);
        let dg = ((rel_diff - 0.08).max(0.0) * 1.1).clamp(0.0, 2.0);
        if dg > 0.0 {
            items.push(InterDowngradeItem {
                name: "loop_radius_match",
                description: format!("Loop radii variation ({:.1}m .. {:.1}m)", min_r, max_r),
                relative_error_pct: rel_diff * 100.0,
                raw_downgrade: dg,
            });
        }
    }

    // 2. Interior Line length matching
    let interior_lines: Vec<f64> = elements
        .iter()
        .filter(|e| e.kind == ElementKind::Line && e.line_length_m > 12.0)
        .map(|e| e.line_length_m)
        .collect();
    if interior_lines.len() >= 2 {
        let mean_l = interior_lines.iter().sum::<f64>() / (interior_lines.len() as f64);
        let max_l = interior_lines.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let min_l = interior_lines.iter().copied().fold(f64::INFINITY, f64::min);
        let rel_diff = (max_l - min_l) / mean_l.max(15.0);
        let dg = ((rel_diff - 0.12).max(0.0) * 0.9).clamp(0.0, 1.8);
        if dg > 0.0 {
            items.push(InterDowngradeItem {
                name: "line_length_match",
                description: format!("Interior line length balance ({:.1}m .. {:.1}m)", min_l, max_l),
                relative_error_pct: rel_diff * 100.0,
                raw_downgrade: dg,
            });
        }
    }

    // 3. Roll rate matching across multiple Roll elements
    let roll_rates: Vec<f64> = elements
        .iter()
        .filter(|e| e.kind == ElementKind::Roll && e.mean_roll_rate_dps > 25.0)
        .map(|e| e.mean_roll_rate_dps)
        .collect();
    if roll_rates.len() >= 2 {
        let mean_rr = roll_rates.iter().sum::<f64>() / (roll_rates.len() as f64);
        let max_rr = roll_rates.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let min_rr = roll_rates.iter().copied().fold(f64::INFINITY, f64::min);
        let rel_diff = (max_rr - min_rr) / mean_rr.max(30.0);
        let dg = ((rel_diff - 0.10).max(0.0) * 0.95).clamp(0.0, 1.5);
        if dg > 0.0 {
            items.push(InterDowngradeItem {
                name: "roll_rate_match",
                description: format!("Multi-element roll rate match ({:.0}°/s .. {:.0}°/s)", min_rr, max_rr),
                relative_error_pct: rel_diff * 100.0,
                raw_downgrade: dg,
            });
        }
    }

    // 4. Speed consistency across elements
    if elements.len() >= 2 {
        let speeds: Vec<f64> = elements.iter().map(|e| e.mean_speed_mps).collect();
        let mean_s = speeds.iter().sum::<f64>() / (speeds.len() as f64);
        let max_s = speeds.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let min_s = speeds.iter().copied().fold(f64::INFINITY, f64::min);
        let rel_diff = (max_s - min_s) / mean_s.max(15.0);
        let dg = ((rel_diff - 0.28).max(0.0) * 0.7).clamp(0.0, 1.2);
        if dg > 0.0 {
            items.push(InterDowngradeItem {
                name: "speed_consistency",
                description: format!("Manoeuvre speed envelope ({:.1} .. {:.1} m/s)", min_s, max_s),
                relative_error_pct: rel_diff * 100.0,
                raw_downgrade: dg,
            });
        }
    }

    let raw_inter_total = items.iter().map(|i| i.raw_downgrade).sum();
    ManoeuvreInterReport {
        items,
        raw_inter_total,
    }
}
