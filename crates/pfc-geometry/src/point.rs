use crate::utils::{cubic_interp_1d, gradient_1d, unwrap_1d};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;
use std::ops::{Add, Div, Index, Mul, Neg, Sub};

/// Vectorized 3D point/vector sequence `[(x, y, z); N]`, matching `geometry.Point`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Point {
    pub data: Vec<[f64; 3]>,
}

pub fn px(value: f64, count: usize) -> Point {
    Point::x_vec(value, count)
}

pub fn py(value: f64, count: usize) -> Point {
    Point::y_vec(value, count)
}

pub fn pz(value: f64, count: usize) -> Point {
    Point::z_vec(value, count)
}

pub fn p0(count: usize) -> Point {
    Point::zeros(count)
}

pub type Points = Point;

impl Point {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            data: vec![[x, y, z]],
        }
    }

    pub fn from_rows(data: Vec<[f64; 3]>) -> Self {
        Self { data }
    }

    pub fn from_columns(x: &[f64], y: &[f64], z: &[f64]) -> Self {
        let n = x.len().max(y.len()).max(z.len());
        if n == 0 {
            return Self { data: Vec::new() };
        }
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            data.push([
                x[i.min(x.len() - 1)],
                y[i.min(y.len() - 1)],
                z[i.min(z.len() - 1)],
            ]);
        }
        Self { data }
    }

    pub fn zeros(count: usize) -> Self {
        Self {
            data: vec![[0.0, 0.0, 0.0]; count.max(1)],
        }
    }

    pub fn x_vec(value: f64, count: usize) -> Self {
        Self {
            data: vec![[value, 0.0, 0.0]; count.max(1)],
        }
    }

    pub fn y_vec(value: f64, count: usize) -> Self {
        Self {
            data: vec![[0.0, value, 0.0]; count.max(1)],
        }
    }

    pub fn z_vec(value: f64, count: usize) -> Self {
        Self {
            data: vec![[0.0, 0.0, value]; count.max(1)],
        }
    }

    pub fn circle_xy(radius: f64, n: usize) -> Self {
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            let theta = 2.0 * PI * (i as f64) / (n as f64);
            data.push([radius * theta.cos(), radius * theta.sin(), 0.0]);
        }
        Self { data }
    }

    pub fn ellipse_xy(a: f64, b: f64, n: usize) -> Self {
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            let theta = 2.0 * PI * (i as f64) / (n as f64);
            data.push([a * theta.cos(), b * theta.sin(), 0.0]);
        }
        Self { data }
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

    pub fn ends(&self) -> Self {
        if self.data.is_empty() {
            return Self::zeros(1);
        }
        Self {
            data: vec![self.data[0], *self.data.last().unwrap()],
        }
    }

    pub fn x(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[0]).collect()
    }

    pub fn y(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[1]).collect()
    }

    pub fn z(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[2]).collect()
    }

    #[inline]
    pub fn x0(&self) -> f64 {
        self.data[0][0]
    }

    #[inline]
    pub fn y0(&self) -> f64 {
        self.data[0][1]
    }

    #[inline]
    pub fn z0(&self) -> f64 {
        self.data[0][2]
    }

    pub fn xy(&self) -> Self {
        Self {
            data: self.data.iter().map(|&[x, y, _]| [x, y, 0.0]).collect(),
        }
    }

    pub fn yz(&self) -> Self {
        Self {
            data: self.data.iter().map(|&[_, y, z]| [0.0, y, z]).collect(),
        }
    }

    pub fn zx(&self) -> Self {
        Self {
            data: self.data.iter().map(|&[x, _, z]| [x, 0.0, z]).collect(),
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

    pub fn radians(&self) -> Self {
        Self {
            data: self
                .data
                .iter()
                .map(|&[x, y, z]| [x.to_radians(), y.to_radians(), z.to_radians()])
                .collect(),
        }
    }

    pub fn degrees(&self) -> Self {
        Self {
            data: self
                .data
                .iter()
                .map(|&[x, y, z]| [x.to_degrees(), y.to_degrees(), z.to_degrees()])
                .collect(),
        }
    }

    pub fn abs_elem(&self) -> Self {
        Self {
            data: self
                .data
                .iter()
                .map(|&[x, y, z]| [x.abs(), y.abs(), z.abs()])
                .collect(),
        }
    }

    /// Euclidean row norms matching Python `abs(point)`.
    pub fn norms(&self) -> Vec<f64> {
        self.data
            .iter()
            .map(|&[x, y, z]| (x * x + y * y + z * z).sqrt())
            .collect()
    }

    pub fn norm0(&self) -> f64 {
        let [x, y, z] = self.data[0];
        (x * x + y * y + z * z).sqrt()
    }

    pub fn scale(&self, value: f64) -> Self {
        Self {
            data: self
                .data
                .iter()
                .map(|&[x, y, z]| {
                    let n = (x * x + y * y + z * z).sqrt();
                    if n <= 1e-15 || !n.is_finite() {
                        [0.0, 0.0, 0.0]
                    } else {
                        let s = value / n;
                        [x * s, y * s, z * s]
                    }
                })
                .collect(),
        }
    }

    pub fn scale_vec(&self, values: &[f64]) -> Self {
        let len = self.len().max(values.len());
        let mut data = Vec::with_capacity(len);
        for i in 0..len {
            let [x, y, z] = self.data[i.min(self.len() - 1)];
            let v = values[i.min(values.len() - 1)];
            let n = (x * x + y * y + z * z).sqrt();
            if n <= 1e-15 || !n.is_finite() {
                data.push([0.0, 0.0, 0.0]);
            } else {
                let s = v / n;
                data.push([x * s, y * s, z * s]);
            }
        }
        Self { data }
    }

    pub fn unit(&self) -> Self {
        self.scale(1.0)
    }

    pub fn dot(&self, other: &Self) -> Vec<f64> {
        let n = self.len().max(other.len());
        (0..n)
            .map(|i| {
                let a = self.data[i.min(self.len() - 1)];
                let b = other.data[i.min(other.len() - 1)];
                a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
            })
            .collect()
    }

    pub fn cross(&self, other: &Self) -> Self {
        let n = self.len().max(other.len());
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            let a = self.data[i.min(self.len() - 1)];
            let b = other.data[i.min(other.len() - 1)];
            data.push([
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]);
        }
        Self { data }
    }

    pub fn cos_angle_between(&self, other: &Self) -> Vec<f64> {
        self.unit().dot(&other.unit())
    }

    pub fn angle_between(&self, other: &Self) -> Vec<f64> {
        self.cos_angle_between(other)
            .into_iter()
            .map(|c| c.clamp(-1.0, 1.0).acos())
            .collect()
    }

    pub fn min_angle_between(&self, other: &Self) -> Vec<f64> {
        self.angle_between(other)
            .into_iter()
            .map(|a| {
                let m = a.rem_euclid(PI);
                m.min(PI - m)
            })
            .collect()
    }

    pub fn scalar_projection(&self, other: &Self) -> Vec<f64> {
        let cos = self.cos_angle_between(other);
        let norms = self.norms();
        let n = cos.len().max(norms.len());
        (0..n)
            .map(|i| cos[i.min(cos.len() - 1)] * norms[i.min(norms.len() - 1)])
            .collect()
    }

    pub fn vector_projection(&self, other: &Self) -> Self {
        let sp = self.scalar_projection(other);
        other.scale_vec(&sp)
    }

    pub fn vector_rejection(&self, other: &Self) -> Self {
        let ab = self.dot(other);
        let bb = other.dot(other);
        let n = ab.len();
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            let a = self.data[i.min(self.len() - 1)];
            let b = other.data[i.min(other.len() - 1)];
            let s = if bb[i].abs() <= 1e-15 {
                0.0
            } else {
                ab[i] / bb[i]
            };
            data.push([a[0] - s * b[0], a[1] - s * b[1], a[2] - s * b[2]]);
        }
        Self { data }
    }

    pub fn is_parallel(&self, other: &Self, tolerance: f64) -> Vec<bool> {
        self.cos_angle_between(other)
            .into_iter()
            .map(|c| (c - 1.0).abs() < tolerance)
            .collect()
    }

    pub fn is_anti_parallel(&self, other: &Self, tolerance: f64) -> Vec<bool> {
        self.cos_angle_between(&(-other.clone()))
            .into_iter()
            .map(|c| (c - 1.0).abs() < tolerance)
            .collect()
    }

    pub fn is_either_parallel(&self, other: &Self, tolerance: f64) -> Vec<bool> {
        self.min_angle_between(other)
            .into_iter()
            .map(|a| a < tolerance)
            .collect()
    }

    pub fn is_perpendicular(&self, other: &Self, tolerance: f64) -> Vec<bool> {
        self.dot(other)
            .into_iter()
            .map(|d| d.abs() < tolerance)
            .collect()
    }

    pub fn angles(&self, p2: &Self) -> Self {
        let cr = self.cross(p2);
        let n1 = self.norms();
        let n2 = p2.norms();
        let n = cr.len();
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            let denom = n1[i.min(n1.len() - 1)] * n2[i.min(n2.len() - 1)];
            if denom <= 1e-15 {
                data.push([0.0, 0.0, 0.0]);
            } else {
                let c = cr.data[i];
                data.push([
                    (c[0] / denom).clamp(-1.0, 1.0).asin(),
                    (c[1] / denom).clamp(-1.0, 1.0).asin(),
                    (c[2] / denom).clamp(-1.0, 1.0).asin(),
                ]);
            }
        }
        Self { data }
    }

    pub fn planar_angles(&self) -> Self {
        Self {
            data: self
                .data
                .iter()
                .map(|&[x, y, z]| [y.atan2(z), z.atan2(x), x.atan2(y)])
                .collect(),
        }
    }

    pub fn bearing(&self) -> Vec<f64> {
        self.data.iter().map(|&[x, y, _]| y.atan2(x)).collect()
    }

    pub fn arbitrary_perpendicular(&self) -> Self {
        let mut cvecs = Vec::with_capacity(self.len());
        for &[x, y, z] in &self.data {
            let ax = x.abs();
            let ay = y.abs();
            let az = z.abs();
            if ax <= ay && ax <= az {
                cvecs.push([1.0, 0.0, 0.0]);
            } else if ay <= ax && ay <= az {
                cvecs.push([0.0, 1.0, 0.0]);
            } else {
                cvecs.push([0.0, 0.0, 1.0]);
            }
        }
        self.cross(&Self { data: cvecs })
    }

    pub fn to_rotation_matrix(&self) -> Vec<[[f64; 3]; 3]> {
        self.data
            .iter()
            .map(|&[rx, ry, rz]| {
                let (sx, cx) = rx.sin_cos();
                let (sy, cy) = ry.sin_cos();
                let (sz, cz) = rz.sin_cos();
                [
                    [cz * cy, cz * sy * sx - cx * sz, cx * cz * sy + sx * sz],
                    [cy * sz, cx * cz + sx * sy * sz, -cz * sx + cx * sy * sz],
                    [-sy, cy * sx, cx * cy],
                ]
            })
            .collect()
    }

    pub fn matrix(&self) -> Vec<[[f64; 3]; 3]> {
        self.data
            .iter()
            .map(|&[x, y, z]| [[x, 0.0, 0.0], [0.0, y, 0.0], [0.0, 0.0, z]])
            .collect()
    }

    pub fn from_matrix(mats: &[[[f64; 3]; 3]]) -> Self {
        Self {
            data: mats.iter().map(|m| [m[0][0], m[1][1], m[2][2]]).collect(),
        }
    }

    pub fn skew_symmetric(&self) -> Vec<[[f64; 3]; 3]> {
        self.data
            .iter()
            .map(|&[x, y, z]| [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
            .collect()
    }

    pub fn remove_outliers(&self, nstds: f64) -> Self {
        if self.data.is_empty() {
            return self.clone();
        }
        let ab = self.norms();
        let valid: Vec<f64> = ab.iter().copied().filter(|v| v.is_finite()).collect();
        if valid.is_empty() {
            return self.clone();
        }
        let mean = valid.iter().sum::<f64>() / (valid.len() as f64);
        let var = valid.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (valid.len() as f64);
        let std = var.sqrt();

        let mut rows: Vec<Option<[f64; 3]>> = self
            .data
            .iter()
            .zip(ab.iter())
            .map(|(&row, &norm)| {
                if !norm.is_finite() || (norm - mean).abs() > nstds * std {
                    None
                } else {
                    Some(row)
                }
            })
            .collect();

        // Forward fill
        let mut last_valid = None;
        for item in &mut rows {
            if item.is_some() {
                last_valid = *item;
            } else if last_valid.is_some() {
                *item = last_valid;
            }
        }
        // Backward fill
        let mut next_valid = None;
        for item in rows.iter_mut().rev() {
            if item.is_some() {
                next_valid = *item;
            } else if next_valid.is_some() {
                *item = next_valid;
            }
        }

        Self {
            data: rows
                .into_iter()
                .map(|r| r.unwrap_or([0.0, 0.0, 0.0]))
                .collect(),
        }
    }

    pub fn unwrap(&self) -> Self {
        let ux = unwrap_1d(&self.x(), PI);
        let uy = unwrap_1d(&self.y(), PI);
        let uz = unwrap_1d(&self.z(), PI);
        Self::from_columns(&ux, &uy, &uz)
    }

    pub fn diff(&self, dt: &[f64], method: &str) -> Self {
        if self.len() <= 1 {
            return Self::zeros(self.len());
        }
        if method == "diff" {
            let mut data = Vec::with_capacity(self.len() - 1);
            for i in 0..self.len() - 1 {
                let dti = dt[i.min(dt.len() - 1)];
                let a = self.data[i];
                let b = self.data[i + 1];
                data.push([(b[0] - a[0]) / dti, (b[1] - a[1]) / dti, (b[2] - a[2]) / dti]);
            }
            Self { data }
        } else {
            let dx = gradient_1d(&self.x(), dt);
            let dy = gradient_1d(&self.y(), dt);
            let dz = gradient_1d(&self.z(), dt);
            Self::from_columns(&dx, &dy, &dz)
        }
    }

    pub fn linterp(&self, index: &[f64], target_ts: &[f64]) -> Self {
        let mut data = Vec::with_capacity(target_ts.len());
        for &t in target_ts {
            if index.is_empty() {
                data.push([0.0, 0.0, 0.0]);
            } else if t <= index[0] || index.len() == 1 {
                data.push(self.data[0]);
            } else if t >= *index.last().unwrap() {
                data.push(*self.data.last().unwrap());
            } else {
                let i = match index.binary_search_by(|v| v.partial_cmp(&t).unwrap_or(std::cmp::Ordering::Equal)) {
                    Ok(idx) => {
                        data.push(self.data[idx]);
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
                let a = self.data[i];
                let b = self.data[i + 1];
                data.push([
                    a[0] + frac * (b[0] - a[0]),
                    a[1] + frac * (b[1] - a[1]),
                    a[2] + frac * (b[2] - a[2]),
                ]);
            }
        }
        Self { data }
    }

    pub fn bspline(&self, index: &[f64], target_ts: &[f64]) -> Self {
        let xs = self.x();
        let ys = self.y();
        let zs = self.z();
        let mut data = Vec::with_capacity(target_ts.len());
        for &t in target_ts {
            data.push([
                cubic_interp_1d(index, &xs, t),
                cubic_interp_1d(index, &ys, t),
                cubic_interp_1d(index, &zs, t),
            ]);
        }
        Self { data }
    }

    pub fn mul_scalars(&self, scalars: &[f64]) -> Self {
        let n = self.len().max(scalars.len());
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            let p = self.data[i.min(self.len() - 1)];
            let s = scalars[i.min(scalars.len() - 1)];
            data.push([p[0] * s, p[1] * s, p[2] * s]);
        }
        Self { data }
    }

    pub fn div_scalars(&self, scalars: &[f64]) -> Self {
        let n = self.len().max(scalars.len());
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            let p = self.data[i.min(self.len() - 1)];
            let s = scalars[i.min(scalars.len() - 1)];
            data.push([p[0] / s, p[1] / s, p[2] / s]);
        }
        Self { data }
    }
}

impl Index<usize> for Point {
    type Output = [f64; 3];
    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}

impl Neg for Point {
    type Output = Self;
    fn neg(self) -> Self::Output {
        Self {
            data: self.data.into_iter().map(|[x, y, z]| [-x, -y, -z]).collect(),
        }
    }
}

impl Neg for &Point {
    type Output = Point;
    fn neg(self) -> Self::Output {
        Point {
            data: self.data.iter().map(|&[x, y, z]| [-x, -y, -z]).collect(),
        }
    }
}

macro_rules! impl_point_binop {
    ($trait:ident, $meth:ident, $op:tt) => {
        impl $trait<&Point> for &Point {
            type Output = Point;
            fn $meth(self, rhs: &Point) -> Point {
                let n = self.len().max(rhs.len());
                let mut data = Vec::with_capacity(n);
                for i in 0..n {
                    let a = self.data[i.min(self.len() - 1)];
                    let b = rhs.data[i.min(rhs.len() - 1)];
                    data.push([a[0] $op b[0], a[1] $op b[1], a[2] $op b[2]]);
                }
                Point { data }
            }
        }

        impl $trait<Point> for Point {
            type Output = Point;
            fn $meth(self, rhs: Point) -> Point {
                (&self).$meth(&rhs)
            }
        }

        impl $trait<&Point> for Point {
            type Output = Point;
            fn $meth(self, rhs: &Point) -> Point {
                (&self).$meth(rhs)
            }
        }

        impl $trait<Point> for &Point {
            type Output = Point;
            fn $meth(self, rhs: Point) -> Point {
                self.$meth(&rhs)
            }
        }

        impl $trait<f64> for &Point {
            type Output = Point;
            fn $meth(self, rhs: f64) -> Point {
                Point {
                    data: self.data.iter().map(|&[x, y, z]| [x $op rhs, y $op rhs, z $op rhs]).collect(),
                }
            }
        }

        impl $trait<f64> for Point {
            type Output = Point;
            fn $meth(self, rhs: f64) -> Point {
                (&self).$meth(rhs)
            }
        }
    };
}

impl_point_binop!(Add, add, +);
impl_point_binop!(Sub, sub, -);
impl_point_binop!(Mul, mul, *);
impl_point_binop!(Div, div, /);

impl Mul<Point> for f64 {
    type Output = Point;
    fn mul(self, rhs: Point) -> Point {
        &rhs * self
    }
}

impl Mul<&Point> for f64 {
    type Output = Point;
    fn mul(self, rhs: &Point) -> Point {
        rhs * self
    }
}
