use crate::point::{p0, px, py, pz, Point};
use crate::quaternion::Quaternion;
use serde::{Deserialize, Serialize};

/// 3D Coordinate Frame representation (origin + 3 orthogonal unit axes), matching `geometry.Coord`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Coord {
    pub origin: Point,
    pub x_axis: Point,
    pub y_axis: Point,
    pub z_axis: Point,
}

impl Coord {
    pub fn from_axes(o: &Point, x: &Point, y: &Point, z: &Point) -> Self {
        let n = o.len().max(x.len()).max(y.len()).max(z.len());
        let xu = x.unit();
        let yu = y.unit();
        let zu = z.unit();
        Self {
            origin: if o.len() == 1 && n > 1 { o.tile(n) } else { o.clone() },
            x_axis: if xu.len() == 1 && n > 1 { xu.tile(n) } else { xu },
            y_axis: if yu.len() == 1 && n > 1 { yu.tile(n) } else { yu },
            z_axis: if zu.len() == 1 && n > 1 { zu.tile(n) } else { zu },
        }
    }

    pub fn from_nothing(count: usize) -> Self {
        Self::from_axes(&p0(count), &px(1.0, count), &py(1.0, count), &pz(1.0, count))
    }

    pub fn zero(count: usize) -> Self {
        Self::from_nothing(count)
    }

    pub fn from_xy(origin: &Point, x_axis: &Point, y_axis: &Point) -> Self {
        let z = x_axis.cross(y_axis);
        let y_orth = z.cross(x_axis);
        Self::from_axes(origin, x_axis, &y_orth, &z)
    }

    pub fn from_yz(origin: &Point, y_axis: &Point, z_axis: &Point) -> Self {
        let x = y_axis.cross(z_axis);
        let z_orth = x.cross(y_axis);
        Self::from_axes(origin, &x, y_axis, &z_orth)
    }

    pub fn from_zx(origin: &Point, z_axis: &Point, x_axis: &Point) -> Self {
        let y = z_axis.cross(x_axis);
        let x_orth = y.cross(z_axis);
        Self::from_axes(origin, &x_orth, &y, z_axis)
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.origin.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.origin.is_empty()
    }

    pub fn rotation_matrix(&self) -> Vec<[[f64; 3]; 3]> {
        let n = self.len();
        (0..n)
            .map(|i| {
                [
                    self.x_axis.data[i.min(self.x_axis.len() - 1)],
                    self.y_axis.data[i.min(self.y_axis.len() - 1)],
                    self.z_axis.data[i.min(self.z_axis.len() - 1)],
                ]
            })
            .collect()
    }

    pub fn inverse_rotation_matrix(&self) -> Vec<[[f64; 3]; 3]> {
        Quaternion::from_rotation_matrix(&self.rotation_matrix())
            .inverse()
            .to_rotation_matrix()
    }

    pub fn rotate(&self, rotation: &Quaternion) -> Self {
        Self::from_axes(
            &self.origin,
            &rotation.transform_point(&self.x_axis),
            &rotation.transform_point(&self.y_axis),
            &rotation.transform_point(&self.z_axis),
        )
    }

    pub fn translate(&self, point: &Point) -> Self {
        Self::from_axes(
            &(&self.origin + point),
            &self.x_axis,
            &self.y_axis,
            &self.z_axis,
        )
    }
}
