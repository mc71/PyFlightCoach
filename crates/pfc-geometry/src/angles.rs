use std::f64::consts::PI;

/// Unwind angle(s) around `center` into `[-pi + center, pi + center]`.
pub fn unwind_scalar(angle: f64, center: f64) -> f64 {
    let turns = ((angle - center) / (2.0 * PI)).round();
    angle - turns * 2.0 * PI
}

pub fn unwind(angles: &[f64], center: f64) -> Vec<f64> {
    angles.iter().map(|&a| unwind_scalar(a, center)).collect()
}

/// Shortest angular difference `a - b` in `[-pi, pi]`.
pub fn difference_scalar(a: f64, b: f64) -> f64 {
    let d1 = a - b;
    let d2 = d1 - 2.0 * PI;
    let d3 = d1 + 2.0 * PI;
    let mut best = d1;
    if d2.abs() < best.abs() {
        best = d2;
    }
    if d3.abs() < best.abs() {
        best = d3;
    }
    best
}

pub fn difference(a: &[f64], b: &[f64]) -> Vec<f64> {
    let n = a.len().max(b.len());
    (0..n)
        .map(|i| {
            let ai = a[i.min(a.len() - 1)];
            let bi = b[i.min(b.len() - 1)];
            difference_scalar(ai, bi)
        })
        .collect()
}

pub fn wrap_to_pi_scalar(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

pub fn wrap_to_pi(angles: &[f64]) -> Vec<f64> {
    angles.iter().map(|&a| wrap_to_pi_scalar(a)).collect()
}
