use crate::maninfo::ManInfo;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PEKind {
    Line,
    Roll,
    Loop,
    Snap,
    Spin,
    Stallturn,
    Tailslide,
}

impl PEKind {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Line => "line",
            Self::Roll => "roll",
            Self::Loop => "loop",
            Self::Snap => "snap",
            Self::Spin => "spin",
            Self::Stallturn => "stallturn",
            Self::Tailslide => "tailslide",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PE {
    pub kind: PEKind,
    pub args: Vec<serde_json::Value>,
    pub kwargs: HashMap<String, serde_json::Value>,
    #[serde(default)]
    pub centred: bool,
}

impl PE {
    pub fn build(
        kind: PEKind,
        args: Vec<serde_json::Value>,
        kwargs: HashMap<String, serde_json::Value>,
        centred: bool,
    ) -> Self {
        Self {
            kind,
            args,
            kwargs,
            centred,
        }
    }

    pub fn with_centred(mut self) -> Self {
        self.centred = true;
        self
    }
}

impl std::fmt::Display for PE {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let args_str: Vec<String> = self.args.iter().map(|v| v.to_string()).collect();
        let kwargs_str: Vec<String> = self
            .kwargs
            .iter()
            .map(|(k, v)| format!("{}={}", k, v))
            .collect();
        let mut inner = args_str;
        inner.extend(kwargs_str);
        let base = format!("{}({})", self.kind.as_str(), inner.join(", "));
        if self.centred {
            write!(f, "centred({})", base)
        } else {
            write!(f, "{}", base)
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum PEOrTag {
    Element(PE),
    Tag(i32),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Figure {
    pub info: ManInfo,
    pub elements: Vec<PEOrTag>,
    pub ndmps: HashMap<String, serde_json::Value>,
    #[serde(default)]
    pub relax_back: bool,
    #[serde(default)]
    pub builder_options: HashMap<String, String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OptionFigure {
    pub figures: Vec<Figure>,
}

impl OptionFigure {
    pub fn info(&self) -> Option<&ManInfo> {
        self.figures.first().map(|f| &f.info)
    }

    pub fn builder_options(&self) -> Option<&HashMap<String, String>> {
        self.figures.first().map(|f| &f.builder_options)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum FigureOrOption {
    Figure(Figure),
    Option(OptionFigure),
}

impl FigureOrOption {
    pub fn info(&self) -> Option<&ManInfo> {
        match self {
            Self::Figure(f) => Some(&f.info),
            Self::Option(o) => o.info(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Sequence {
    pub name: String,
    pub rules: String,
    pub figures: Vec<FigureOrOption>,
    #[serde(default)]
    pub options: HashMap<String, String>,
}

impl Sequence {
    pub fn get_by_index(&self, idx: usize) -> Option<&FigureOrOption> {
        self.figures.get(idx)
    }

    pub fn get_by_short_name(&self, short_name: &str) -> Option<&FigureOrOption> {
        self.figures
            .iter()
            .find(|f| f.info().map(|i| i.short_name.as_str()) == Some(short_name))
    }

    pub fn k_factors(&self) -> HashMap<String, f64> {
        let mut map = HashMap::new();
        for fig in &self.figures {
            if let Some(info) = fig.info() {
                map.insert(info.short_name.clone(), info.k);
            }
        }
        map
    }
}
