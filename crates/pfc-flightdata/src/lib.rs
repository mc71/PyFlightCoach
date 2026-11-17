pub mod alignment;
pub mod environment;
pub mod flight;
pub mod labels;
pub mod origin;
pub mod state;

pub use alignment::{dtw, Alignment};
pub use environment::{Attack, Coefficients, Constants, Environment, Flow};
pub use flight::{
    flightmode_name, origin_from_channel_switch, origin_from_pilot_centre_logs, Flight,
};
pub use labels::{Label, LabelGroup, LabelGroups};
pub use origin::Origin;
pub use state::{kinematic_interp_pva, State};

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use std::path::PathBuf;

    fn test_data_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../packages/FlightData/test/data")
    }

    #[test]
    fn test_origin_f3a_zone() {
        let f3a_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../packages/FlightData/test/EmailedBox.f3a");
        let org = Origin::from_f3a_zone(&f3a_path).expect("parse EmailedBox.f3a");
        assert!(!org.name.is_empty());
        let serialized = org.to_f3a_zone_string();
        let roundtrip = Origin::from_f3a_zone_str(&serialized).expect("roundtrip .f3a");
        assert_abs_diff_eq!(org.heading, roundtrip.heading, epsilon = 1e-4);
    }

    #[test]
    fn test_flight_and_state_from_json() {
        let json_path = test_data_dir().join("p23_flight.json");
        let flight = Flight::from_json_file(&json_path).expect("load p23_flight.json");
        assert!(flight.len() > 100);

        let state = State::from_flight(&flight, None).expect("State::from_flight");
        assert_eq!(state.len(), flight.len());
        assert!(state.duration() > 10.0);

        // Test Environment and Flow from State
        let env = Environment::from_flight(&flight, None);
        let flow = Flow::from_state(&state, &env);
        assert_eq!(flow.aspd.len(), state.len());
    }

    #[test]
    fn test_flightline_from_bin_logs() {
        let script_dir = test_data_dir().join("script_tests");
        let c6_log = Flight::from_log_file(script_dir.join("c6_on_0001.BIN"), "pos", 0)
            .expect("parse c6_on_0001.BIN");
        let org1 = origin_from_channel_switch(&c6_log, 6).expect("origin from channel 6");
        assert!(org1.lat().abs() > 1.0);
    }
}
