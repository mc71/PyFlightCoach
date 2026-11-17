use crate::point::{px, pz, Point};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;
use std::ops::{Index, Mul, Neg};

/// Vectorized 3D rotation quaternion sequence `[(w, x, y, z); N]`, matching `geometry.Quaternion`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Quaternion {
    pub data: Vec<[f64; 4]>,
}

pub fn q0(count: usize) -> Quaternion {
    Quaternion::zero(count)
}

pub fn euler(eul: &Point) -> Quaternion {
    Quaternion::from_euler(eul)
}

pub fn euler_angles(roll: f64, pitch: f64, yaw: f64) -> Quaternion {
    Quaternion::from_euler(&Point::new(roll, pitch, yaw))
}

pub fn euldeg(eul_deg: &Point) -> Quaternion {
    Quaternion::from_euler(&eul_deg.radians())
}

pub fn euldeg_angles(roll_deg: f64, pitch_deg: f64, yaw_deg: f64) -> Quaternion {
    Quaternion::from_euler(&Point::new(roll_deg, pitch_deg, yaw_deg).radians())
}

impl Quaternion {
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self {
            data: vec![[w, x, y, z]],
        }
    }

    pub fn from_rows(data: Vec<[f64; 4]>) -> Self {
        Self { data }
    }

    pub fn from_columns(w: &[f64], x: &[f64], y: &[f64], z: &[f64]) -> Self {
        let n = w.len().max(x.len()).max(y.len()).max(z.len());
        if n == 0 {
            return Self { data: Vec::new() };
        }
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            data.push([
                w[i.min(w.len() - 1)],
                x[i.min(x.len() - 1)],
                y[i.min(y.len() - 1)],
                z[i.min(z.len() - 1)],
            ]);
        }
        Self { data }
    }

    pub fn zero(count: usize) -> Self {
        Self {
            data: vec![[1.0, 0.0, 0.0, 0.0]; count.max(1)],
        }
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

    pub fn tile(&self, count: usize) -> Self {
        let mut data = Vec::with_capacity(self.data.len() * count);
        for _ in 0..count {
            data.extend_from_slice(&self.data);
        }
        Self { data }
    }

    pub fn concatenate(items: &[Self]) -> Self {
        let mut data = Vec::new();
        for item in items {
            data.extend_from_slice(&item.data);
        }
        Self { data }
    }

    pub fn w(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[0]).collect()
    }

    pub fn x(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[1]).collect()
    }

    pub fn y(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[2]).collect()
    }

    pub fn z(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[3]).collect()
    }

    pub fn xyzw(&self) -> Vec<[f64; 4]> {
        self.data
            .iter()
            .map(|&[w, x, y, z]| [x, y, z, w])
            .collect()
    }

    pub fn axis(&self) -> Point {
        Point::from_rows(self.data.iter().map(|&[_, x, y, z]| [x, y, z]).collect())
    }

    pub fn norms(&self) -> Vec<f64> {
        self.data
            .iter()
            .map(|&[w, x, y, z]| (w * w + x * x + y * y + z * z).sqrt())
            .collect()
    }

    pub fn norm(&self) -> Self {
        Self {
            data: self
                .data
                .iter()
                .map(|&[w, x, y, z]| {
                    let n = (w * w + x * x + y * y + z * z).sqrt();
                    if n <= 1e-15 {
                        [1.0, 0.0, 0.0, 0.0]
                    } else {
                        [w / n, x / n, y / n, z / n]
                    }
                })
                .collect(),
        }
    }

    pub fn conjugate(&self) -> Self {
        Self {
            data: self
                .data
                .iter()
                .map(|&[w, x, y, z]| [w, -x, -y, -z])
                .collect(),
        }
    }

    pub fn inverse(&self) -> Self {
        self.conjugate().norm()
    }

    #[inline]
    fn hamilton_single(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
        let [aw, ax, ay, az] = a;
        let [bw, bx, by, bz] = b;
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + bw * ax + ay * bz - az * by,
            aw * by + bw * ay + az * bx - ax * bz,
            aw * bz + bw * az + ax * by - ay * bx,
        ]
    }

    pub fn transform_point(&self, point: &Point) -> Point {
        let inv = self.inverse();
        let n = self.len().max(point.len());
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let q = self.data[i.min(self.len() - 1)];
            let qi = inv.data[i.min(inv.len() - 1)];
            let [px, py, pz] = point.data[i.min(point.len() - 1)];
            let p_quat = [0.0, px, py, pz];
            let tmp = Self::hamilton_single(q, p_quat);
            let res = Self::hamilton_single(tmp, qi);
            out.push([res[1], res[2], res[3]]);
        }
        Point::from_rows(out)
    }

    pub fn from_euler(eul: &Point) -> Self {
        let unwrapped = eul.unwrap();
        let mut data = Vec::with_capacity(unwrapped.len());
        for &[rx, ry, rz] in &unwrapped.data {
            let (sx, cx) = (rx * 0.5).sin_cos();
            let (sy, cy) = (ry * 0.5).sin_cos();
            let (sz, cz) = (rz * 0.5).sin_cos();
            data.push([
                cy * cz * cx + sy * sz * sx,
                cy * cz * sx - sy * sz * cx,
                sy * cz * cx + cy * sz * sx,
                cy * sz * cx - sy * cz * sx,
            ]);
        }
        Self { data }
    }

    pub fn to_euler(&self) -> Point {
        let mut data = Vec::with_capacity(self.len());
        for &[w, x, y, z] in &self.data {
            let sinr_cosp = 2.0 * (w * x + y * z);
            let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
            let mut roll = sinr_cosp.atan2(cosr_cosp);

            let sinp = 2.0 * (w * y - z * x);
            let mut pitch = sinp.clamp(-1.0, 1.0).asin();

            let siny_cosp = 2.0 * (w * z + x * y);
            let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
            let mut yaw = siny_cosp.atan2(cosy_cosp);

            if sinp.abs() >= 0.9999 {
                pitch = (PI / 2.0).copysign(sinp);
                yaw = 0.0;
                roll = 2.0 * x.atan2(w);
            }
            data.push([roll, pitch, yaw]);
        }
        Point::from_rows(data)
    }

    pub fn from_axis_angle(axangles: &Point) -> Self {
        let mut data = Vec::with_capacity(axangles.len());
        for &[vx, vy, vz] in &axangles.data {
            let theta = (vx * vx + vy * vy + vz * vz).sqrt();
            if theta < 1e-6 {
                data.push([1.0, 0.0, 0.0, 0.0]);
            } else {
                let half = 0.5 * theta;
                let s = half.sin() / theta;
                data.push([half.cos(), vx * s, vy * s, vz * s]);
            }
        }
        Self { data }
    }

    fn to_axis_angle_raw(&self) -> Point {
        let mut data = Vec::with_capacity(self.len());
        for &[w, x, y, z] in &self.data {
            let wc = w.clamp(-1.0, 1.0);
            let theta = 2.0 * wc.acos();
            let s = (1.0 - wc * wc).max(0.0).sqrt();
            if s < 1e-6 {
                data.push([x * theta, y * theta, z * theta]);
            } else {
                let fac = theta / s;
                data.push([
                    if (x * fac).is_finite() { x * fac } else { 0.0 },
                    if (y * fac).is_finite() { y * fac } else { 0.0 },
                    if (z * fac).is_finite() { z * fac } else { 0.0 },
                ]);
            }
        }
        Point::from_rows(data)
    }

    pub fn to_axis_angle(&self) -> Point {
        let a = self.to_axis_angle_raw();
        let b = (-self).to_axis_angle_raw();
        let na = a.norms();
        let nb = b.norms();
        let mut data = Vec::with_capacity(self.len());
        for i in 0..self.len() {
            if na[i] <= nb[i] {
                data.push(a.data[i]);
            } else {
                data.push(b.data[i]);
            }
        }
        Point::from_rows(data)
    }

    pub fn axis_rates(q: &Self, qdot: &Self) -> Point {
        let dq = (qdot * &q.conjugate()).norm();
        dq.to_axis_angle()
    }

    pub fn body_axis_rates(q: &Self, qdot: &Self) -> Point {
        let dq = (&q.conjugate() * qdot).norm();
        dq.to_axis_angle()
    }

    pub fn rotate(&self, rate: &Point) -> Self {
        (&Self::from_axis_angle(rate) * self).norm()
    }

    pub fn body_rotate(&self, rate: &Point) -> Self {
        (self * &Self::from_axis_angle(rate)).norm()
    }

    pub fn diff(&self, dt: &[f64]) -> Point {
        let n = self.len();
        if n <= 1 {
            return Point::zeros(n);
        }
        let scale = (n as f64) / ((n - 1) as f64);
        let q0_slice = self.slice(0, n - 1);
        let q1_slice = self.slice(1, n);
        let rates = Self::axis_rates(&q0_slice, &q1_slice);
        let mut rows = Vec::with_capacity(n);
        for i in 0..n - 1 {
            let dti = dt[i.min(dt.len() - 1)] * scale;
            let r = rates.data[i];
            rows.push([r[0] / dti, r[1] / dti, r[2] / dti]);
        }
        rows.push(*rows.last().unwrap());
        Point::from_rows(rows)
    }

    pub fn body_diff(&self, dt: &[f64]) -> Point {
        let n = self.len();
        if n <= 1 {
            return Point::zeros(n);
        }
        let scale = (n as f64) / ((n - 1) as f64);
        let q0_slice = self.slice(0, n - 1);
        let q1_slice = self.slice(1, n);
        let rates = Self::body_axis_rates(&q0_slice, &q1_slice);
        let mut rows = Vec::with_capacity(n);
        for i in 0..n - 1 {
            let dti = dt[i.min(dt.len() - 1)] * scale;
            let r = rates.data[i];
            rows.push([r[0] / dti, r[1] / dti, r[2] / dti]);
        }
        rows.push(*rows.last().unwrap());
        Point::from_rows(rows)
    }

    pub fn to_rotation_matrix(&self) -> Vec<[[f64; 3]; 3]> {
        let normed = self.norm();
        normed
            .data
            .iter()
            .map(|&[s, x, y, z]| {
                // Matches Python's `.T` on the (3, 3, N) array
                [
                    [
                        1.0 - 2.0 * (y * y + z * z),
                        2.0 * (x * y + s * z),
                        2.0 * (x * z - s * y),
                    ],
                    [
                        2.0 * (x * y - s * z),
                        1.0 - 2.0 * (x * x + z * z),
                        2.0 * (y * z + s * x),
                    ],
                    [
                        2.0 * (x * z + s * y),
                        2.0 * (y * z - s * x),
                        1.0 - 2.0 * (x * x + y * y),
                    ],
                ]
            })
            .collect()
    }

    pub fn from_rotation_matrix(matrices: &[[[f64; 3]; 3]]) -> Self {
        let mut data = Vec::with_capacity(matrices.len());
        for mat in matrices {
            // Note: Python transposes `m = matrix.conj().transpose()`, so m[r][c] = mat[c][r]
            let m00 = mat[0][0];
            let m01 = mat[1][0];
            let m02 = mat[2][0];
            let m10 = mat[0][1];
            let m11 = mat[1][1];
            let m12 = mat[2][1];
            let m20 = mat[0][2];
            let m21 = mat[1][2];
            let m22 = mat[2][2];

            let (w, x, y, z) = if m22 < 0.0 {
                if m00 > m11 {
                    let t = 1.0 + m00 - m11 - m22;
                    let s = 0.5 / t.max(1e-15).sqrt();
                    ((m12 - m21) * s, t * s, (m01 + m10) * s, (m20 + m02) * s)
                } else {
                    let t = 1.0 - m00 + m11 - m22;
                    let s = 0.5 / t.max(1e-15).sqrt();
                    ((m20 - m02) * s, (m01 + m10) * s, t * s, (m12 + m21) * s)
                }
            } else if m00 < -m11 {
                let t = 1.0 - m00 - m11 + m22;
                let s = 0.5 / t.max(1e-15).sqrt();
                ((m01 - m10) * s, (m20 + m02) * s, (m12 + m21) * s, t * s)
            } else {
                let t = 1.0 + m00 + m11 + m22;
                let s = 0.5 / t.max(1e-15).sqrt();
                (t * s, (m12 - m21) * s, (m20 - m02) * s, (m01 - m10) * s)
            };
            // Python divides by -2.0 * sqrt(t), which negates all 4 components (equivalent rotation)
            data.push([-w, -x, -y, -z]);
        }
        Self { data }
    }

    pub fn closest_principal(&self) -> Self {
        let eul = self.to_euler();
        let step = 0.5 * PI;
        let rounded = Point::from_rows(
            eul.data
                .iter()
                .map(|&[r, p, y]| [
                    (r / step).round() * step,
                    (p / step).round() * step,
                    (y / step).round() * step,
                ])
                .collect(),
        );
        Self::from_euler(&rounded)
    }

    pub fn is_inverted(&self) -> Vec<bool> {
        let tz = self.transform_point(&pz(1.0, 1));
        tz.data.iter().map(|r| r[2] > 0.0).collect()
    }

    pub fn bearing(&self, p: Option<&Point>) -> Vec<f64> {
        let default_p = px(1.0, 1);
        let target = p.unwrap_or(&default_p);
        self.transform_point(target).bearing()
    }

    pub fn slerp_pair(q0: [f64; 4], q1: [f64; 4], t: f64) -> [f64; 4] {
        let mut b = q1;
        let mut dot = q0[0] * b[0] + q0[1] * b[1] + q0[2] * b[2] + q0[3] * b[3];
        if dot < 0.0 {
            b = [-b[0], -b[1], -b[2], -b[3]];
            dot = -dot;
        }
        if dot > 0.9995 {
            let mut res = [
                q0[0] + t * (b[0] - q0[0]),
                q0[1] + t * (b[1] - q0[1]),
                q0[2] + t * (b[2] - q0[2]),
                q0[3] + t * (b[3] - q0[3]),
            ];
            let n = (res[0] * res[0] + res[1] * res[1] + res[2] * res[2] + res[3] * res[3]).sqrt();
            if n > 1e-15 {
                res = [res[0] / n, res[1] / n, res[2] / n, res[3] / n];
            }
            return res;
        }
        let theta_0 = dot.clamp(-1.0, 1.0).acos();
        let theta = theta_0 * t;
        let sin_theta = theta.sin();
        let sin_theta_0 = theta_0.sin();
        let s0 = theta.cos() - dot * sin_theta / sin_theta_0;
        let s1 = sin_theta / sin_theta_0;
        [
            s0 * q0[0] + s1 * b[0],
            s0 * q0[1] + s1 * b[1],
            s0 * q0[2] + s1 * b[2],
            s0 * q0[3] + s1 * b[3],
        ]
    }

    pub fn slerp(&self, index: &[f64], target_ts: &[f64]) -> Self {
        let normed = self.norm();
        let mut data = Vec::with_capacity(target_ts.len());
        for &t in target_ts {
            if index.is_empty() {
                data.push([1.0, 0.0, 0.0, 0.0]);
            } else if t <= index[0] || index.len() == 1 {
                data.push(normed.data[0]);
            } else if t >= *index.last().unwrap() {
                data.push(*normed.data.last().unwrap());
            } else {
                let i = match index.binary_search_by(|v| v.partial_cmp(&t).unwrap_or(std::cmp::Ordering::Equal)) {
                    Ok(idx) => {
                        data.push(normed.data[idx]);
                        continue;
                    }
                    Err(idx) => idx.saturating_sub(1).min(index.len() - 2),
                };
                let denom = index[i + 1] - index[i];
                let frac = if denom.abs() < 1e-15 {
                    0.0
                } else {
                    (t - index[i]) / denom
                };
                data.push(Self::slerp_pair(normed.data[i], normed.data[i + 1], frac));
            }
        }
        Self { data }
    }

    pub fn squad(p: &Self, a: &Self, b: &Self, q: &Self, t: &[f64]) -> Self {
        let n = p.len().max(q.len()).max(t.len());
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            let ti = t[i.min(t.len() - 1)];
            let pi = p.data[i.min(p.len() - 1)];
            let ai = a.data[i.min(a.len() - 1)];
            let bi = b.data[i.min(b.len() - 1)];
            let qi = q.data[i.min(q.len() - 1)];
            let slerp_pq = Self::slerp_pair(pi, qi, ti);
            let slerp_ab = Self::slerp_pair(ai, bi, ti);
            data.push(Self::slerp_pair(slerp_pq, slerp_ab, 2.0 * ti * (1.0 - ti)));
        }
        Self { data }
    }
}

impl Index<usize> for Quaternion {
    type Output = [f64; 4];
    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}

impl Neg for Quaternion {
    type Output = Self;
    fn neg(self) -> Self::Output {
        Self {
            data: self
                .data
                .into_iter()
                .map(|[w, x, y, z]| [-w, -x, -y, -z])
                .collect(),
        }
    }
}

impl Neg for &Quaternion {
    type Output = Quaternion;
    fn neg(self) -> Self::Output {
        Quaternion {
            data: self
                .data
                .iter()
                .map(|&[w, x, y, z]| [-w, -x, -y, -z])
                .collect(),
        }
    }
}

impl Mul<&Quaternion> for &Quaternion {
    type Output = Quaternion;
    fn mul(self, rhs: &Quaternion) -> Quaternion {
        let n = self.len().max(rhs.len());
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            let a = self.data[i.min(self.len() - 1)];
            let b = rhs.data[i.min(rhs.len() - 1)];
            data.push(Quaternion::hamilton_single(a, b));
        }
        Quaternion { data }
    }
}

impl Mul<Quaternion> for Quaternion {
    type Output = Quaternion;
    fn mul(self, rhs: Quaternion) -> Quaternion {
        (&self) * (&rhs)
    }
}

impl Mul<&Quaternion> for Quaternion {
    type Output = Quaternion;
    fn mul(self, rhs: &Quaternion) -> Quaternion {
        (&self) * rhs
    }
}

impl Mul<Quaternion> for &Quaternion {
    type Output = Quaternion;
    fn mul(self, rhs: Quaternion) -> Quaternion {
        self * (&rhs)
    }
}
