use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IndexDirection {
    Forward,
    Backward,
}

/// Find exact or linearly-interpolated floating-point index of `value` in 1D monotonic array `arr`.
pub fn get_index(
    arr: &[f64],
    value: f64,
    direction: IndexDirection,
    increasing: Option<bool>,
) -> Result<f64, String> {
    if arr.is_empty() {
        return Err("Empty array in get_index".to_string());
    }
    if arr.len() == 1 {
        if (arr[0] - value).abs() < 1e-12 {
            return Ok(0.0);
        }
        return Err(format!("Value {} out of bounds for single-element array", value));
    }

    // Check exact match first
    let exact_matches: Vec<usize> = arr
        .iter()
        .enumerate()
        .filter_map(|(i, &v)| if (v - value).abs() == 0.0 { Some(i) } else { None })
        .collect();
    if !exact_matches.is_empty() {
        return Ok(match direction {
            IndexDirection::Forward => exact_matches[0] as f64,
            IndexDirection::Backward => *exact_matches.last().unwrap() as f64,
        });
    }

    let is_inc = increasing.unwrap_or_else(|| arr.last().unwrap() >= &arr[0]);
    if is_inc {
        if value < arr[0] || value > *arr.last().unwrap() {
            return Err(format!(
                "Value {} outside array range [{}, {}]",
                value,
                arr[0],
                arr.last().unwrap()
            ));
        }
        for i in 0..arr.len() - 1 {
            if value >= arr[i] && value <= arr[i + 1] {
                let denom = arr[i + 1] - arr[i];
                if denom.abs() < 1e-15 {
                    return Ok(i as f64);
                }
                return Ok(i as f64 + (value - arr[i]) / denom);
            }
        }
    } else {
        if value > arr[0] || value < *arr.last().unwrap() {
            return Err(format!(
                "Value {} outside decreasing array range [{}, {}]",
                value,
                arr[0],
                arr.last().unwrap()
            ));
        }
        for i in 0..arr.len() - 1 {
            if value <= arr[i] && value >= arr[i + 1] {
                let denom = arr[i + 1] - arr[i];
                if denom.abs() < 1e-15 {
                    return Ok(i as f64);
                }
                return Ok(i as f64 + (value - arr[i]) / denom);
            }
        }
    }
    Err(format!("Could not locate value {} in array", value))
}

/// Inverse of `get_index`: given a floating-point index, linearly interpolate in `arr`.
pub fn get_value(arr: &[f64], index: f64) -> Result<f64, String> {
    if arr.is_empty() {
        return Err("Empty array in get_value".to_string());
    }
    let idx = if index < 0.0 {
        arr.len() as f64 + index
    } else {
        index
    };
    if idx < 0.0 || idx > (arr.len() - 1) as f64 + 1e-12 {
        return Err(format!(
            "Index {} out of bounds for array of length {}",
            index,
            arr.len()
        ));
    }
    let idx = idx.clamp(0.0, (arr.len() - 1) as f64);
    let i0 = idx.trunc() as usize;
    let frac = idx.fract();
    if frac.abs() < 1e-12 || i0 >= arr.len() - 1 {
        Ok(arr[i0])
    } else {
        Ok(arr[i0] + frac * (arr[i0 + 1] - arr[i0]))
    }
}

/// Slice `index` by floating-point `[start, stop]` indices, prepending/appending interpolated boundary values.
pub fn apply_index_slice(index: &[f64], start: Option<f64>, stop: Option<f64>) -> Result<Vec<f64>, String> {
    if index.is_empty() {
        return Ok(Vec::new());
    }
    let s = start.unwrap_or(0.0);
    let s = if s < 0.0 { index.len() as f64 + s } else { s };
    let e = stop.unwrap_or((index.len() - 1) as f64);
    let e = if e < 0.0 { index.len() as f64 + e } else { e };

    let start_val = get_value(index, s)?;
    let stop_val = get_value(index, e)?;

    let first_int = s.floor() as usize + 1;
    let last_int = e.ceil() as usize;

    let mut out = vec![start_val];
    if first_int < last_int && first_int < index.len() {
        out.extend_from_slice(&index[first_int..last_int.min(index.len())]);
    }
    if (e - s).abs() > 1e-12 {
        out.push(stop_val);
    }
    Ok(out)
}

/// Unwrap 1D phase angles with discontinuity `discont` (typically `PI`), matching `numpy.unwrap`.
pub fn unwrap_1d(data: &[f64], discont: f64) -> Vec<f64> {
    if data.is_empty() {
        return Vec::new();
    }
    let mut out = Vec::with_capacity(data.len());
    out.push(data[0]);
    let mut corr = 0.0;
    let period = 2.0 * PI;
    for i in 1..data.len() {
        let dd = data[i] - data[i - 1];
        let ddmod = (dd + PI).rem_euclid(period) - PI;
        let ph_correct = if ddmod == -PI && dd > 0.0 {
            PI
        } else {
            ddmod
        } - dd;
        if dd.abs() >= discont {
            corr += ph_correct;
        }
        out.push(data[i] + corr);
    }
    out
}

/// Numerical gradient matching `numpy.gradient(f) / dt`.
pub fn gradient_1d(f: &[f64], dt: &[f64]) -> Vec<f64> {
    let n = f.len();
    if n <= 1 {
        return vec![0.0; n];
    }
    let mut out = vec![0.0; n];
    out[0] = (f[1] - f[0]) / dt[0];
    for i in 1..n - 1 {
        out[i] = 0.5 * (f[i + 1] - f[i - 1]) / dt[i.min(dt.len() - 1)];
    }
    out[n - 1] = (f[n - 1] - f[n - 2]) / dt[(n - 1).min(dt.len() - 1)];
    out
}

/// Zero-phase forward-backward second-order biquad cascade low-pass filter (Butterworth approximation).
pub fn butterworth_lowpass_filtfilt(signal: &[f64], cutoff_hz: f64, sample_rate_hz: f64, passes: usize) -> Vec<f64> {
    let n = signal.len();
    if n < 4 || cutoff_hz <= 0.0 || sample_rate_hz <= 0.0 || cutoff_hz >= 0.5 * sample_rate_hz {
        return signal.to_vec();
    }
    let omega = (PI * cutoff_hz / sample_rate_hz).tan();
    let omega2 = omega * omega;
    let sqrt2 = std::f64::consts::SQRT_2;
    let norm = 1.0 / (1.0 + sqrt2 * omega + omega2);
    let b0 = omega2 * norm;
    let b1 = 2.0 * b0;
    let b2 = b0;
    let a1 = 2.0 * (omega2 - 1.0) * norm;
    let a2 = (1.0 - sqrt2 * omega + omega2) * norm;

    let apply_one_way = |input: &[f64]| -> Vec<f64> {
        let mut y = vec![0.0; input.len()];
        let x0 = input[0];
        y[0] = x0;
        y[1] = b0 * input[1] + b1 * x0 + b2 * x0 - a1 * x0 - a2 * x0;
        for i in 2..input.len() {
            y[i] = b0 * input[i] + b1 * input[i - 1] + b2 * input[i - 2]
                - a1 * y[i - 1]
                - a2 * y[i - 2];
        }
        y
    };

    let mut cur = signal.to_vec();
    for _ in 0..passes.max(1) {
        let mut fwd = apply_one_way(&cur);
        fwd.reverse();
        let mut bwd = apply_one_way(&fwd);
        bwd.reverse();
        cur = bwd;
    }
    cur
}

/// Catmull-Rom / natural cubic spline 1D interpolation matching smooth trajectory requirements.
pub fn cubic_interp_1d(xs: &[f64], ys: &[f64], target_x: f64) -> f64 {
    let n = xs.len();
    if n == 0 {
        return 0.0;
    }
    if n == 1 || target_x <= xs[0] {
        return ys[0];
    }
    if target_x >= xs[n - 1] {
        return ys[n - 1];
    }
    // Find segment i such that xs[i] <= target_x <= xs[i+1]
    let i = match xs.binary_search_by(|v| v.partial_cmp(&target_x).unwrap_or(std::cmp::Ordering::Equal)) {
        Ok(idx) => return ys[idx],
        Err(idx) => idx.saturating_sub(1).min(n - 2),
    };
    let h = xs[i + 1] - xs[i];
    if h.abs() < 1e-15 {
        return ys[i];
    }
    let t = (target_x - xs[i]) / h;
    if n < 4 {
        return ys[i] + t * (ys[i + 1] - ys[i]);
    }
    // Estimate tangents m0, m1
    let slope = |k: usize| -> f64 {
        let dk = xs[k + 1] - xs[k];
        if dk.abs() < 1e-15 {
            0.0
        } else {
            (ys[k + 1] - ys[k]) / dk
        }
    };
    let m0 = if i == 0 {
        slope(0)
    } else {
        0.5 * (slope(i - 1) + slope(i))
    };
    let m1 = if i + 1 == n - 1 {
        slope(n - 2)
    } else {
        0.5 * (slope(i) + slope(i + 1))
    };
    let t2 = t * t;
    let t3 = t2 * t;
    let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    let h10 = t3 - 2.0 * t2 + t;
    let h01 = -2.0 * t3 + 3.0 * t2;
    let h11 = t3 - t2;
    h00 * ys[i] + h10 * h * m0 + h01 * ys[i + 1] + h11 * h * m1
}
