use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::f64::consts::PI;

pub struct MBTags;
impl MBTags {
    pub const CENTRE: i32 = 0;
}

pub fn c45() -> f64 {
    45.0_f64.to_radians().cos()
}

pub fn r(turns: &[f64]) -> Vec<f64> {
    turns.iter().map(|t| 2.0 * PI * t).collect()
}

macro_rules! impl_enum_str {
    ($name:ident { $($variant:ident => $str_val:expr, $float_val:expr),+ $(,)? }) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        pub enum $name {
            $($variant),+
        }

        impl $name {
            pub fn value(self) -> f64 {
                match self {
                    $(Self::$variant => $float_val),+
                }
            }

            pub fn as_str(self) -> &'static str {
                match self {
                    $(Self::$variant => $str_val),+
                }
            }

            pub fn from_name(s: &str) -> Option<Self> {
                match s {
                    $($str_val => Some(Self::$variant),)+
                    _ => None,
                }
            }

            pub fn from_f64(v: f64) -> Option<Self> {
                $(
                    if (v - ($float_val)).abs() < 1e-6 {
                        return Some(Self::$variant);
                    }
                )+
                None
            }
        }

        impl Serialize for $name {
            fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                serializer.serialize_str(self.as_str())
            }
        }

        impl<'de> Deserialize<'de> for $name {
            fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
                let val = serde_json::Value::deserialize(deserializer)?;
                match val {
                    serde_json::Value::String(s) => Self::from_name(&s)
                        .ok_or_else(|| serde::de::Error::custom(format!("Unknown {}: {}", stringify!($name), s))),
                    serde_json::Value::Number(n) => {
                        let f = n.as_f64().ok_or_else(|| serde::de::Error::custom("Invalid number"))?;
                        Self::from_f64(f).ok_or_else(|| serde::de::Error::custom(format!("Unknown {} value: {}", stringify!($name), f)))
                    }
                    _ => Err(serde::de::Error::custom("Expected string or number")),
                }
            }
        }
    };
}

pub fn deserialize_opt_enum<'de, D, T>(deserializer: D) -> Result<Option<T>, D::Error>
where
    D: Deserializer<'de>,
    T: for<'a> Deserialize<'a>,
{
    let val = Option::<serde_json::Value>::deserialize(deserializer)?;
    match val {
        None | Some(serde_json::Value::Null) => Ok(None),
        Some(v) => Ok(serde_json::from_value::<T>(v).ok()),
    }
}

impl_enum_str!(Orientation {
    UPRIGHT => "UPRIGHT", PI,
    INVERTED => "INVERTED", 0.0,
});

impl_enum_str!(Heading {
    RTOL => "RTOL", PI,
    LTOR => "LTOR", 0.0,
    OUTTOIN => "OUTTOIN", 1.5 * PI,
    INTOOUT => "INTOOUT", 0.5 * PI,
});

impl Heading {
    pub fn values() -> [f64; 4] {
        [
            Self::RTOL.value(),
            Self::LTOR.value(),
            Self::OUTTOIN.value(),
            Self::INTOOUT.value(),
        ]
    }

    pub fn infer(bearing: f64) -> Result<Self, String> {
        for h in [Self::RTOL, Self::LTOR, Self::OUTTOIN, Self::INTOOUT] {
            let k = ((4.0 * (bearing - h.value())).abs() / (2.0 * PI)).round() as i64;
            if k % 4 == 0 {
                return Ok(h);
            }
        }
        Err(format!("Could not infer Heading from bearing {}", bearing))
    }

    pub fn reverse(self) -> Self {
        match self {
            Self::RTOL => Self::LTOR,
            Self::LTOR => Self::RTOL,
            Self::OUTTOIN => Self::INTOOUT,
            Self::INTOOUT => Self::OUTTOIN,
        }
    }
}

impl_enum_str!(Direction {
    UPWIND => "UPWIND", 1.0,
    DOWNWIND => "DOWNWIND", -1.0,
    CROSS => "CROSS", 0.0,
});

impl Direction {
    pub fn parse_heading(heading: Heading, wind: Heading) -> Self {
        if heading == wind {
            Self::DOWNWIND
        } else if matches!(heading, Heading::INTOOUT | Heading::OUTTOIN) {
            Self::CROSS
        } else {
            Self::UPWIND
        }
    }

    pub fn wind_swap_heading(self, d_or_w: Heading) -> Heading {
        if self == Self::DOWNWIND {
            d_or_w.reverse()
        } else {
            d_or_w
        }
    }

    pub fn parse(s: &str) -> Result<Self, String> {
        match s.chars().next().map(|c| c.to_ascii_lowercase()) {
            Some('u') => Ok(Self::UPWIND),
            Some('d') => Ok(Self::DOWNWIND),
            Some('c') => Ok(Self::CROSS),
            _ => Err(format!("Cannot parse Direction from '{}'", s)),
        }
    }
}

impl_enum_str!(Height {
    BTM => "BTM", 0.2,
    MID => "MID", 0.6,
    TOP => "TOP", 1.0,
});

impl_enum_str!(Position {
    CENTRE => "CENTRE", 0.0,
    END => "END", 1.0,
});

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct BoxLocation {
    #[serde(default, deserialize_with = "deserialize_opt_enum")]
    pub height: Option<Height>,
    #[serde(default, deserialize_with = "deserialize_opt_enum")]
    pub direction: Option<Direction>,
    #[serde(default, deserialize_with = "deserialize_opt_enum")]
    pub orientation: Option<Orientation>,
}

impl BoxLocation {
    pub fn build(
        height: Option<Height>,
        direction: Option<Direction>,
        orientation: Option<Orientation>,
    ) -> Self {
        Self {
            height,
            direction,
            orientation,
        }
    }
}
