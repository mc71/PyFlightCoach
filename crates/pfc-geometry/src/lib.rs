pub mod air;
pub mod angles;
pub mod coord;
pub mod gps;
pub mod mass;
pub mod point;
pub mod quaternion;
pub mod time;
pub mod transformation;
pub mod utils;

pub use air::{get_rho, Air, GAMMA, R_GAS};
pub use angles::{difference, difference_scalar, unwind, unwind_scalar, wrap_to_pi, wrap_to_pi_scalar};
pub use coord::Coord;
pub use gps::{safecos, ERAD, GPS, LOCFAC};
pub use mass::Mass;
pub use point::{p0, px, py, pz, Point, Points};
pub use quaternion::{euler, euler_angles, euldeg, euldeg_angles, q0, Quaternion};
pub use time::Time;
pub use transformation::Transformation;
pub use utils::{
    apply_index_slice, butterworth_lowpass_filtfilt, cubic_interp_1d, get_index, get_value,
    gradient_1d, unwrap_1d, IndexDirection,
};

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use std::f64::consts::PI;

    #[test]
    fn test_angles() {
        assert_abs_diff_eq!(unwind_scalar(3.0 * PI, 0.0), -PI, epsilon = 1e-10);
        assert_abs_diff_eq!(unwind_scalar(3.0 * PI, PI), PI, epsilon = 1e-10);
        assert_abs_diff_eq!(unwind_scalar(-PI, PI), PI, epsilon = 1e-10);
        assert_abs_diff_eq!(difference_scalar(0.1, 2.0 * PI - 0.1), 0.2, epsilon = 1e-10);
    }

    #[test]
    fn test_point_operations() {
        let a = px(1.0, 1);
        let b = py(1.0, 1);
        let c = a.cross(&b);
        assert_abs_diff_eq!(c.z0(), 1.0, epsilon = 1e-10);

        let zero_scaled = p0(1).scale(5.0);
        assert_eq!(zero_scaled.data[0], [0.0, 0.0, 0.0]);

        let ang = px(-1.0, 1).angle_between(&Point::new(1.0, 1.0, 0.0))[0];
        assert_abs_diff_eq!(ang, 3.0 * PI / 4.0, epsilon = 1e-10);

        assert!(px(2.0, 1).is_parallel(&px(5.0, 1), 1e-6)[0]);
        assert!(px(2.0, 1).is_perpendicular(&py(3.0, 1), 1e-6)[0]);

        // Outlier removal test: 100 zeros with [1000, 1000, 1000] spike at index 50
        let mut rows = vec![[0.0, 0.0, 0.0]; 100];
        rows[50] = [1000.0, 1000.0, 1000.0];
        let pt = Point::from_rows(rows).remove_outliers(2.0);
        assert_abs_diff_eq!(pt.data[50][0], 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_quaternion_and_rates() {
        let q = euler_angles(0.0, 0.0, PI / 2.0);
        let p = px(1.0, 1);
        let tp = q.transform_point(&p);
        assert_abs_diff_eq!(tp.x0(), 0.0, epsilon = 1e-10);
        assert_abs_diff_eq!(tp.y0(), 1.0, epsilon = 1e-10);
        assert_abs_diff_eq!(tp.z0(), 0.0, epsilon = 1e-10);

        // Axis rates vs body axis rates at yaw = pi/2 with 5 deg roll
        let droll = 5.0_f64.to_radians();
        let q1 = euler_angles(0.0, 0.0, PI / 2.0);
        let q2 = euler_angles(droll, 0.0, PI / 2.0);
        let wrate = Quaternion::axis_rates(&q1, &q2);
        let brate = Quaternion::body_axis_rates(&q1, &q2);
        assert_abs_diff_eq!(wrate.x0(), 0.0, epsilon = 1e-7);
        assert_abs_diff_eq!(wrate.y0(), droll, epsilon = 1e-7);
        assert_abs_diff_eq!(brate.x0(), droll, epsilon = 1e-7);
        assert_abs_diff_eq!(brate.y0(), 0.0, epsilon = 1e-7);

        // Axis angle roundtrip
        let aa = Point::new(0.2, -0.3, 0.5);
        let q_aa = Quaternion::from_axis_angle(&aa);
        let aa2 = q_aa.to_axis_angle();
        assert_abs_diff_eq!(aa.x0(), aa2.x0(), epsilon = 1e-10);
        assert_abs_diff_eq!(aa.y0(), aa2.y0(), epsilon = 1e-10);
        assert_abs_diff_eq!(aa.z0(), aa2.z0(), epsilon = 1e-10);
    }

    #[test]
    fn test_coord_and_transform() {
        let c = Coord::from_nothing(1).rotate(&euler_angles(0.0, 0.0, PI / 2.0));
        assert_abs_diff_eq!(c.x_axis.y0(), 1.0, epsilon = 1e-10);
        assert_abs_diff_eq!(c.y_axis.x0(), -1.0, epsilon = 1e-10);
        assert_abs_diff_eq!(c.z_axis.z0(), 1.0, epsilon = 1e-10);

        let c1 = Coord::from_nothing(1);
        let c2 = Coord::from_nothing(1)
            .translate(&Point::new(10.0, -5.0, 2.0))
            .rotate(&euler_angles(0.1, 0.2, 0.3));
        let tr = Transformation::from_coords(&c1, &c2);
        let c2_reconstructed = tr.coord(Some(&c1));
        assert_abs_diff_eq!(c2_reconstructed.origin.x0(), c2.origin.x0(), epsilon = 1e-6);
        assert_abs_diff_eq!(c2_reconstructed.origin.y0(), c2.origin.y0(), epsilon = 1e-6);
        assert_abs_diff_eq!(c2_reconstructed.origin.z0(), c2.origin.z0(), epsilon = 1e-6);
        assert_abs_diff_eq!(c2_reconstructed.x_axis.x0(), c2.x_axis.x0(), epsilon = 1e-6);
        assert_abs_diff_eq!(c2_reconstructed.y_axis.y0(), c2.y_axis.y0(), epsilon = 1e-6);
        assert_abs_diff_eq!(c2_reconstructed.z_axis.z0(), c2.z_axis.z0(), epsilon = 1e-6);
    }

    #[test]
    fn test_gps() {
        let centre = GPS::new(52.542375, -1.631038, 0.0);
        let pilot = GPS::new(52.542264, -1.631817, 0.0);
        let vec = &centre - &pilot;
        assert_abs_diff_eq!(vec.norm0(), 54.167, epsilon = 1e-2);
        assert_abs_diff_eq!(vec.x0(), 12.3563, epsilon = 1e-3);
        assert_abs_diff_eq!(vec.y0(), 52.7393, epsilon = 1e-3);

        let offset_back = pilot.offset(&vec);
        assert_abs_diff_eq!(offset_back.lat0(), centre.lat0(), epsilon = 1e-7);
        assert_abs_diff_eq!(offset_back.long0(), centre.long0(), epsilon = 1e-7);
    }

    #[test]
    fn test_mass_and_utils() {
        let m = Mass::point(1.0).offset(&px(1.0, 1));
        let mat = m.matrix()[0];
        assert_abs_diff_eq!(mat[0][0], 0.0, epsilon = 1e-10);
        assert_abs_diff_eq!(mat[1][1], 1.0, epsilon = 1e-10);
        assert_abs_diff_eq!(mat[2][2], 1.0, epsilon = 1e-10);

        let arr = [0.0, 1.0, 2.0, 3.0];
        let idx = get_index(&arr, 1.5, IndexDirection::Forward, None).unwrap();
        assert_abs_diff_eq!(idx, 1.5, epsilon = 1e-10);
        let val = get_value(&arr, idx).unwrap();
        assert_abs_diff_eq!(val, 1.5, epsilon = 1e-10);
    }
}
