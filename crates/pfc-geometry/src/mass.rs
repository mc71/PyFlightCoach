use crate::point::Point;
use serde::{Deserialize, Serialize};

/// Mass and 3x3 moment of inertia tensor `[m, xx, xy, xz, yx, yy, yz, zx, zy, zz]`, matching `geometry.Mass`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Mass {
    pub data: Vec<[f64; 10]>,
}

impl Mass {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        m: f64,
        xx: f64,
        xy: f64,
        xz: f64,
        yx: f64,
        yy: f64,
        yz: f64,
        zx: f64,
        zy: f64,
        zz: f64,
    ) -> Self {
        Self {
            data: vec![[m, xx, xy, xz, yx, yy, yz, zx, zy, zz]],
        }
    }

    pub fn from_matrix(m: f64, mat: [[f64; 3]; 3]) -> Self {
        Self::new(
            m,
            mat[0][0],
            mat[0][1],
            mat[0][2],
            mat[1][0],
            mat[1][1],
            mat[1][2],
            mat[2][0],
            mat[2][1],
            mat[2][2],
        )
    }

    pub fn from_principal(m: f64, xx: f64, yy: f64, zz: f64) -> Self {
        Self::new(m, xx, 0.0, 0.0, 0.0, yy, 0.0, 0.0, 0.0, zz)
    }

    pub fn from_double_symmetry(m: f64, xxyyzz: f64) -> Self {
        Self::from_principal(m, xxyyzz, xxyyzz, xxyyzz)
    }

    pub fn point(m: f64) -> Self {
        Self::from_double_symmetry(m, 0.0)
    }

    pub fn sphere(m: f64, r: f64) -> Self {
        Self::from_double_symmetry(m, 0.4 * m * r * r)
    }

    pub fn cuboid(m: f64, lx: f64, ly: f64, lz: f64) -> Self {
        let xx = (m / 12.0) * (ly * ly + lz * lz);
        let yy = (m / 12.0) * (lz * lz + lx * lx);
        let zz = (m / 12.0) * (lx * lx + ly * ly);
        Self::from_principal(m, xx, yy, zz)
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.data.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    pub fn m(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[0]).collect()
    }

    #[inline]
    pub fn m0(&self) -> f64 {
        self.data[0][0]
    }

    pub fn matrix(&self) -> Vec<[[f64; 3]; 3]> {
        self.data
            .iter()
            .map(|r| {
                [
                    [r[1], r[2], r[3]],
                    [r[4], r[5], r[6]],
                    [r[7], r[8], r[9]],
                ]
            })
            .collect()
    }

    /// Parallel axis theorem (Steiner's theorem) offset by `v`.
    pub fn offset(&self, v: &Point) -> Self {
        let n = self.len().max(v.len());
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            let r = self.data[i.min(self.len() - 1)];
            let [vx, vy, vz] = v.data[i.min(v.len() - 1)];
            let m = r[0];
            data.push([
                m,
                r[1] + m * (vy * vy + vz * vz),
                r[2] - m * vx * vy,
                r[3] - m * vx * vz,
                r[4] - m * vx * vy,
                r[5] + m * (vz * vz + vx * vx),
                r[6] - m * vy * vz,
                r[7] - m * vx * vz,
                r[8] - m * vy * vz,
                r[9] + m * (vx * vx + vy * vy),
            ]);
        }
        Self { data }
    }

    pub fn momentum(&self, v: &Point) -> Point {
        v.mul_scalars(&self.m())
    }

    pub fn angular_momentum(&self, rvel: &Point) -> Point {
        let mats = self.matrix();
        let n = mats.len().max(rvel.len());
        let mut rows = Vec::with_capacity(n);
        for i in 0..n {
            let m = mats[i.min(mats.len() - 1)];
            let w = rvel.data[i.min(rvel.len() - 1)];
            rows.push([
                m[0][0] * w[0] + m[0][1] * w[1] + m[0][2] * w[2],
                m[1][0] * w[0] + m[1][1] * w[1] + m[1][2] * w[2],
                m[2][0] * w[0] + m[2][1] * w[1] + m[2][2] * w[2],
            ]);
        }
        Point::from_rows(rows)
    }
}
