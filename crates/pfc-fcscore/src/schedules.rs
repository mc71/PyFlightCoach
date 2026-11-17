use serde::Serialize;

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ScheduleManoeuvreDef {
    pub index: usize,
    pub short_name: &'static str,
    pub full_name: &'static str,
    pub k_factor: f64,
    pub is_centered: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ScheduleCatalogEntry {
    pub category: &'static str,
    pub name: &'static str,
    pub manoeuvres: &'static [ScheduleManoeuvreDef],
}

/// Official FAI F3A P25 Preliminary Schedule (17 scored manoeuvres, total K = 60).
pub const F3A_P25_MANOEUVRES: &[ScheduleManoeuvreDef] = &[
    ScheduleManoeuvreDef { index: 1, short_name: "trgle", full_name: "Triangle Loop with 2/4 Pt Roll & Half Roll", k_factor: 3.0, is_centered: true },
    ScheduleManoeuvreDef { index: 2, short_name: "hSqL", full_name: "Half Square Loop with Half Roll", k_factor: 2.0, is_centered: false },
    ScheduleManoeuvreDef { index: 3, short_name: "sqL", full_name: "Square Loop on Corner with 1/2 Rolls", k_factor: 5.0, is_centered: true },
    ScheduleManoeuvreDef { index: 4, short_name: "fig9", full_name: "Figure 9 with Half Roll", k_factor: 3.0, is_centered: false },
    ScheduleManoeuvreDef { index: 5, short_name: "rollC", full_name: "Roll Combination (4 Consecutive 1/4, Opp 2/4)", k_factor: 4.0, is_centered: true },
    ScheduleManoeuvreDef { index: 6, short_name: "stall", full_name: "Stall Turn with Consecutive 1/2 Rolls", k_factor: 3.0, is_centered: false },
    ScheduleManoeuvreDef { index: 7, short_name: "dImm", full_name: "Double Immelmann with Snap & Half Roll", k_factor: 4.0, is_centered: true },
    ScheduleManoeuvreDef { index: 8, short_name: "hB", full_name: "Humpty Bump with 2/4 Pt Roll & Half Roll", k_factor: 3.0, is_centered: false },
    ScheduleManoeuvreDef { index: 9, short_name: "loop", full_name: "Loop with Integrated Half Rolls in Top 90°", k_factor: 5.0, is_centered: true },
    ScheduleManoeuvreDef { index: 10, short_name: "hSqL2", full_name: "Half Square Loop on Corner", k_factor: 2.0, is_centered: false },
    ScheduleManoeuvreDef { index: 11, short_name: "hClov", full_name: "Half Cloverleaf with 1/2 Rolls", k_factor: 5.0, is_centered: true },
    ScheduleManoeuvreDef { index: 12, short_name: "rEt", full_name: "Reverse Cuban 8 from Top", k_factor: 4.0, is_centered: false },
    ScheduleManoeuvreDef { index: 13, short_name: "iSpin", full_name: "Inverted Spin (2.5 Turns)", k_factor: 3.0, is_centered: true },
    ScheduleManoeuvreDef { index: 14, short_name: "tHat", full_name: "Top Hat with 2/4 Pt Roll", k_factor: 3.0, is_centered: false },
    ScheduleManoeuvreDef { index: 15, short_name: "Z", full_name: "Figure Z with Half Roll", k_factor: 4.0, is_centered: true },
    ScheduleManoeuvreDef { index: 16, short_name: "Com", full_name: "Comet with 2/4 Pt Roll", k_factor: 3.0, is_centered: false },
    ScheduleManoeuvreDef { index: 17, short_name: "figS", full_name: "Figure S with Rolls", k_factor: 5.0, is_centered: true },
];

/// Official FAI F3A P23 Preliminary Schedule (17 scored manoeuvres).
pub const F3A_P23_MANOEUVRES: &[ScheduleManoeuvreDef] = &[
    ScheduleManoeuvreDef { index: 1, short_name: "tHat", full_name: "Top Hat with 2/4 Pt Roll, Half Roll", k_factor: 4.0, is_centered: true },
    ScheduleManoeuvreDef { index: 2, short_name: "hB", full_name: "Half Cuban 8 with Wingover", k_factor: 3.0, is_centered: false },
    ScheduleManoeuvreDef { index: 3, short_name: "oLoop", full_name: "Opposite Half Rolls on Loop", k_factor: 5.0, is_centered: true },
    ScheduleManoeuvreDef { index: 4, short_name: "fB", full_name: "Figure 9 on Corner", k_factor: 3.0, is_centered: false },
    ScheduleManoeuvreDef { index: 5, short_name: "4ptR", full_name: "4-Point Roll & Opposite Roll", k_factor: 4.0, is_centered: true },
    ScheduleManoeuvreDef { index: 6, short_name: "sTurn", full_name: "Stall Turn with 1/4 Rolls", k_factor: 3.0, is_centered: false },
    ScheduleManoeuvreDef { index: 7, short_name: "dImm", full_name: "Double Immelmann with Rolls", k_factor: 4.0, is_centered: true },
    ScheduleManoeuvreDef { index: 8, short_name: "sB", full_name: "Shark Tooth with Half Rolls", k_factor: 3.0, is_centered: false },
    ScheduleManoeuvreDef { index: 9, short_name: "gLoop", full_name: "Golf Ball with Half Roll", k_factor: 5.0, is_centered: true },
    ScheduleManoeuvreDef { index: 10, short_name: "hSqL", full_name: "Half Square Loop", k_factor: 2.0, is_centered: false },
    ScheduleManoeuvreDef { index: 11, short_name: "dK", full_name: "Double Key", k_factor: 5.0, is_centered: true },
    ScheduleManoeuvreDef { index: 12, short_name: "hC8", full_name: "Half Reverse Cuban 8", k_factor: 4.0, is_centered: false },
    ScheduleManoeuvreDef { index: 13, short_name: "Spin", full_name: "Spin with 2.5 Turns", k_factor: 3.0, is_centered: true },
    ScheduleManoeuvreDef { index: 14, short_name: "humB", full_name: "Humpty Bump", k_factor: 3.0, is_centered: false },
    ScheduleManoeuvreDef { index: 15, short_name: "rLoop", full_name: "Rolling Circle / Loop", k_factor: 5.0, is_centered: true },
    ScheduleManoeuvreDef { index: 16, short_name: "hClov", full_name: "Half Cloverleaf", k_factor: 3.0, is_centered: false },
    ScheduleManoeuvreDef { index: 17, short_name: "figM", full_name: "Figure M with 3/4 Pt Rolls", k_factor: 5.0, is_centered: true },
];

pub fn lookup_schedule(name: &str) -> &'static [ScheduleManoeuvreDef] {
    let lower = name.to_ascii_lowercase();
    if lower.contains("p23") {
        F3A_P23_MANOEUVRES
    } else {
        F3A_P25_MANOEUVRES
    }
}

pub fn lookup_manoeuvre_def(
    schedule_name: &str,
    man_index_1based: usize,
    man_name: &str,
) -> ScheduleManoeuvreDef {
    let sched = lookup_schedule(schedule_name);
    let clean_name = man_name.trim_end_matches(|c: char| c.is_ascii_digit());
    if let Some(found) = sched
        .iter()
        .find(|m| m.index == man_index_1based || m.short_name.eq_ignore_ascii_case(man_name) || m.short_name.eq_ignore_ascii_case(clean_name))
    {
        return found.clone();
    }
    let is_centered = man_index_1based % 2 == 1;
    let default_k = if man_name.eq_ignore_ascii_case("tkOf") || man_name.eq_ignore_ascii_case("lnd") {
        0.0
    } else if is_centered {
        4.0
    } else {
        3.0
    };
    ScheduleManoeuvreDef {
        index: man_index_1based,
        short_name: "man",
        full_name: "Aerobatic Manoeuvre",
        k_factor: default_k,
        is_centered,
    }
}
