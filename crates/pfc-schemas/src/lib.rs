pub mod ajson;
pub mod aresti;
pub mod fcj;
pub mod ma;
pub mod maninfo;
pub mod positioning;
pub mod sdef;
pub mod sinfo;

pub use ajson::AJson;
pub use aresti::{Figure, FigureOrOption, OptionFigure, PEKind, PE, Sequence};
pub use fcj::{
    Data, El, FCSResult, HumanResult, Man, ManResult, Origin, Parameters, Score, ScoreProperties,
    ScoreValues, View, FCJ,
};
pub use ma::{FAVersion, MA};
pub use maninfo::ManInfo;
pub use positioning::{BoxLocation, Direction, Heading, Height, MBTags, Orientation, Position};
pub use sdef::{DirectionDefinition, MDef, MOption, SDefFile};
pub use sinfo::{ManDetails, ScheduleInfo};

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn test_data_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../packages/Schemas/tests/data")
    }

    #[test]
    fn test_enumstr_and_boxloc_serialize() {
        assert_eq!(Height::MID, Height::MID);
        assert!((Height::MID.value() - 0.6).abs() < 1e-12);

        let bloc = BoxLocation {
            height: Some(Height::MID),
            direction: Some(Direction::UPWIND),
            orientation: None,
        };
        let json = serde_json::to_string(&bloc).unwrap();
        assert!(json.contains("\"MID\""));
        assert!(json.contains("\"UPWIND\""));
        let decoded: BoxLocation = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, bloc);
    }

    #[test]
    fn test_fcj_validate() {
        let path = test_data_dir().join("fc_json.json");
        let content = std::fs::read_to_string(&path).expect("read fc_json.json");
        let parsed: FCJ = serde_json::from_str(&content).expect("parse FCJ");
        assert_eq!(parsed.version, "1.3");
        assert_eq!(parsed.id().as_deref(), Some("00000109"));
        assert!(!parsed.mans.is_empty());
        assert!(!parsed.data.is_empty());
    }

    #[test]
    fn test_ajson_validate() {
        let path = test_data_dir().join("flight.ajson");
        let content = std::fs::read_to_string(&path).expect("read flight.ajson");
        let parsed: AJson = serde_json::from_str(&content).expect("parse AJson");
        assert!(parsed.isComp);
        assert!(!parsed.mans.is_empty());
        assert_eq!(parsed.mans[0].name, "trgle");
    }
}
