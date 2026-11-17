pub mod inter;
pub mod intra;
pub mod positioning;
pub mod schedules;
pub mod segmentation;

pub use inter::{evaluate_manoeuvre_inter, InterDowngradeItem, ManoeuvreInterReport};
pub use intra::{evaluate_element_intra, scale_downgrade, ElementIntraReport, IntraDowngradeItem};
pub use positioning::{evaluate_manoeuvre_positioning, ManoeuvrePositioningReport};
pub use schedules::{
    lookup_manoeuvre_def, lookup_schedule, ScheduleCatalogEntry, ScheduleManoeuvreDef,
    F3A_P23_MANOEUVRES, F3A_P25_MANOEUVRES,
};
pub use segmentation::{
    auto_segment_flight_manoeuvres, classify_element_slice, segment_manoeuvre_elements,
    ElementKind, SegmentedElement,
};

use pfc_flightdata::State;
use pfc_schemas::{El, FCSResult, ManResult, Score, ScoreProperties, ScoreValues};
use serde::Serialize;

pub const RUST_FCSCORE_VERSION: &str = "rust-fcscore-1.0.0";

/// Detailed FCScore grading report for a single manoeuvre, including element-level intra
/// measurements, inter-element consistency, box positioning, and the standard `ManResult`.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ManoeuvreScoreReport {
    pub manoeuvre_index: usize,
    pub short_name: String,
    pub full_name: String,
    pub k_factor: f64,
    pub is_centered: bool,
    pub elements: Vec<ElementIntraReport>,
    pub inter: ManoeuvreInterReport,
    pub positioning: ManoeuvrePositioningReport,
    pub man_result: ManResult,
}

impl ManoeuvreScoreReport {
    pub fn score_for(&self, props: ScoreProperties) -> ScoreValues {
        self.man_result
            .get_score(Some(props))
            .cloned()
            .unwrap_or(ScoreValues {
                intra: 0.0,
                inter: 0.0,
                positioning: 0.0,
                total: 10.0,
            })
    }
}

/// Complete flight grading report produced by the Rust `pfc-fcscore` engine.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FlightScoreReport {
    pub fa_version: String,
    pub schedule_name: String,
    pub manoeuvres: Vec<ManoeuvreScoreReport>,
    pub fcs_result: FCSResult,
}

/// Score a single manoeuvre `State` and produce a `ManoeuvreScoreReport` with all 6
/// `(difficulty: 1..=3, truncate: false/true)` combinations matching `FCSResult`.
pub fn score_manoeuvre(
    man_state: &State,
    schedule_name: &str,
    manoeuvre_index: usize,
    man_name: &str,
    k_override: Option<f64>,
    preset_els: Option<&[El]>,
) -> ManoeuvreScoreReport {
    let mdef = lookup_manoeuvre_def(schedule_name, manoeuvre_index, man_name);
    let k_factor = k_override.unwrap_or(mdef.k_factor);
    let segmented = segment_manoeuvre_elements(man_state, man_name, preset_els);

    let mut elements = Vec::with_capacity(segmented.len());
    for (idx, seg) in segmented.iter().enumerate() {
        let prev_kind = if idx > 0 {
            Some(segmented[idx - 1].kind)
        } else {
            None
        };
        let next_kind = segmented.get(idx + 1).map(|s| s.kind);
        elements.push(evaluate_element_intra(man_state, seg, prev_kind, next_kind));
    }

    let inter = evaluate_manoeuvre_inter(&elements);

    let entry_stop_idx = segmented
        .first()
        .filter(|e| e.kind == ElementKind::EntryLine && segmented.len() > 2)
        .map(|e| e.stop_idx)
        .unwrap_or(0);
    let exit_start_idx = segmented
        .last()
        .filter(|e| e.kind == ElementKind::ExitLine && segmented.len() > 2)
        .map(|e| e.start_idx)
        .unwrap_or_else(|| man_state.len());

    let positioning = evaluate_manoeuvre_positioning(
        man_state,
        mdef.is_centered,
        entry_stop_idx,
        exit_start_idx,
    );

    let els: Vec<El> = segmented.iter().map(|s| s.to_el()).collect();
    let mut results = Vec::with_capacity(6);

    for difficulty in [1, 2, 3] {
        for truncate in [false, true] {
            let intra_dg: f64 = elements
                .iter()
                .flat_map(|el| el.items.iter())
                .map(|it| scale_downgrade(it.raw_downgrade, difficulty, truncate))
                .sum();
            let inter_dg = inter.scaled_total(difficulty, truncate);
            let pos_dg = positioning.scaled_total(difficulty, truncate);
            let total = (10.0 - intra_dg - inter_dg - pos_dg).clamp(0.0, 10.0);

            results.push(Score {
                score: ScoreValues {
                    intra: intra_dg,
                    inter: inter_dg,
                    positioning: pos_dg,
                    total,
                },
                properties: ScoreProperties {
                    difficulty,
                    truncate,
                },
            });
        }
    }

    ManoeuvreScoreReport {
        manoeuvre_index,
        short_name: man_name.to_string(),
        full_name: mdef.full_name.to_string(),
        k_factor,
        is_centered: mdef.is_centered,
        elements,
        inter,
        positioning,
        man_result: ManResult { els, results },
    }
}

/// Automatically segment (if unlabelled) and score an entire flight `State` against `schedule_name`.
pub fn score_flight_state(state: &mut State, schedule_name: &str) -> FlightScoreReport {
    auto_segment_flight_manoeuvres(state, schedule_name);
    let man_names = state.manoeuvre_names();

    let mut manoeuvres = Vec::new();
    let mut manresults: Vec<Option<ManResult>> = Vec::with_capacity(man_names.len());

    for (idx, mname) in man_names.iter().enumerate() {
        let lower = mname.to_ascii_lowercase();
        if lower == "tkof" || lower == "lnd" || lower == "takeoff" || lower == "landing" {
            manresults.push(None);
            continue;
        }
        if let Some(sliced) = state.slice_manoeuvre(mname) {
            if sliced.len() >= 4 {
                let report = score_manoeuvre(&sliced, schedule_name, idx, mname, None, None);
                manresults.push(Some(report.man_result.clone()));
                manoeuvres.push(report);
            } else {
                manresults.push(None);
            }
        } else {
            manresults.push(None);
        }
    }

    FlightScoreReport {
        fa_version: RUST_FCSCORE_VERSION.to_string(),
        schedule_name: schedule_name.to_string(),
        manoeuvres,
        fcs_result: FCSResult {
            fa_version: RUST_FCSCORE_VERSION.to_string(),
            manresults,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pfc_schemas::FCJ;

    #[test]
    fn test_fcscore_on_sample_p25_flight() {
        let raw_json = include_str!("../../../packages/Schemas/tests/data/fc_json.json");
        let fcj: FCJ = serde_json::from_str(raw_json).expect("valid FCJ");
        let mut st = State::from_fc_json(&fcj).expect("State from FCJ");

        let report = score_flight_state(&mut st, "P25");
        assert_eq!(report.manoeuvres.len(), 17);

        for m in &report.manoeuvres {
            assert!(m.k_factor > 0.0);
            assert!(!m.elements.is_empty());
            let hard = m.score_for(ScoreProperties {
                difficulty: 3,
                truncate: false,
            });
            let easy = m.score_for(ScoreProperties {
                difficulty: 1,
                truncate: false,
            });
            assert!(hard.total >= 0.0 && hard.total <= 10.0);
            assert!(easy.total >= hard.total - 1e-9);
        }
    }
}
