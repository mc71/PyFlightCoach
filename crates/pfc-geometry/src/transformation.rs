use crate::coord::Coord;
use crate::point::{p0, Point};
use crate::quaternion::{q0, Quaternion};
use serde::{Deserialize, Serialize};

/// Rigid-body SE(3) pose combining translation (`Point`) and rotation (`Quaternion`), matching `geometry.Transformation`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Transformation {
    pub p: Point,
    pub q: Quaternion,
}

impl Transformation {
    pub fn new(p: Point, q: Quaternion) -> Self {
        Self::build(&p, &q)
    }

    pub fn build(p: &Point, q: &Quaternion) -> Self {
        let n = p.len().max(q.len());
        let p_out = if p.len() == 1 && n > 1 {
            p.tile(n)
        } else {
            p.clone()
        };
        let q_out = if q.len() == 1 && n > 1 {
            q.tile(n)
        } else {
            q.clone()
        };
        Self { p: p_out, q: q_out }
    }

    pub fn zero(count: usize) -> Self {
        Self {
            p: p0(count),
            q: q0(count),
        }
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.p.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.p.is_empty()
    }

    pub fn row(&self, idx: usize) -> Self {
        Self {
            p: self.p.row(idx),
            q: self.q.row(idx),
        }
    }

    pub fn pos(&self) -> &Point {
        &self.p
    }

    pub fn att(&self) -> &Quaternion {
        &self.q
    }

    pub fn from_coords(coord_a: &Coord, coord_b: &Coord) -> Self {
        let q1 = Quaternion::from_rotation_matrix(&coord_b.rotation_matrix()).inverse();
        let q2 = Quaternion::from_rotation_matrix(&coord_a.rotation_matrix());
        Self::build(&(&coord_b.origin - &coord_a.origin), &(-(&q1 * &q2)))
    }

    pub fn from_coord(coord: &Coord) -> Self {
        Self::from_coords(&Coord::from_nothing(1), coord)
    }

    pub fn rotate_point(&self, pin: &Point) -> Point {
        self.q.transform_point(pin)
    }

    pub fn rotate_quat(&self, qin: &Quaternion) -> Quaternion {
        &self.q * qin
    }

    pub fn translate(&self, point: &Point) -> Point {
        point + &self.p
    }

    /// Rotate `point` by `self.q` then translate by `self.p`.
    pub fn point(&self, point: &Point) -> Point {
        self.translate(&self.rotate_point(point))
    }

    pub fn coord(&self, coord: Option<&Coord>) -> Coord {
        let default_c = Coord::zero(1);
        let c = coord.unwrap_or(&default_c);
        c.translate(&self.p).rotate(&self.q)
    }

    pub fn offset_point(&self, p: &Point) -> Self {
        Self::build(&(&self.p + p), &self.q)
    }

    pub fn offset_transform(&self, t: &Self) -> Self {
        Self::build(&(&self.p + &t.p), &(&self.q * &t.q))
    }

    pub fn to_matrix(&self) -> [[f64; 4]; 4] {
        let r = self.q.to_rotation_matrix()[0];
        let t = self.p.data[0];
        [
            [r[0][0], r[0][1], r[0][2], 0.0],
            [r[1][0], r[1][1], r[1][2], 0.0],
            [r[2][0], r[2][1], r[2][2], 0.0],
            [t[0], t[1], t[2], 1.0],
        ]
    }
}

impl Default for Transformation {
    fn default() -> Self {
        Self::zero(1)
    }
}
