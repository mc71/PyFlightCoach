use crate::flight::Flight;
use crate::origin::Origin;
use crate::state::State;
use pfc_geometry::{euler, p0, Air, Mass, Point, Time};
use serde::{Deserialize, Serialize};

/// Atmospheric environment (`Air`) and 3D wind vector (`Point` in Box frame) over time.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Environment {
    pub time: Time,
    pub atm: Air,
    pub wind: Point,
}

impl Environment {
    pub fn from_constructs(time: Time, atm: Option<Air>, wind: Option<Point>) -> Self {
        let n = time.len();
        Self {
            time,
            atm: atm.unwrap_or_else(|| Air::iso_sea_level(n)),
            wind: wind.unwrap_or_else(|| p0(n)),
        }
    }

    pub fn zero(t: &[f64]) -> Self {
        Self::from_constructs(Time::from_t(t), None, None)
    }

    pub fn from_flight(flight: &Flight, origin: Option<&Origin>) -> Self {
        let org = origin.unwrap_or(&flight.origin);
        let time = Time::from_t(flight.time_flight());
        let n = time.len();

        let atm = if let (Some(p), Some(t_c)) = (
            flight.col("air_pressure"),
            flight.col("air_temperature"),
        ) {
            let t_k: Vec<f64> = t_c.iter().map(|&c| c + 273.15).collect();
            Air::from_pt(p, &t_k)
        } else {
            Air::iso_sea_level(n)
        };

        let wind = if let (Some(wn), Some(we)) = (flight.col("wind_N"), flight.col("wind_E")) {
            let zeros = vec![0.0; n];
            let ned_wind = Point::from_columns(wn, we, &zeros);
            org.rotation().transform_point(&ned_wind)
        } else {
            p0(n)
        };

        Self { time, atm, wind }
    }
}

/// Angle of attack (`alpha`), sideslip (`beta`), and dynamic pressure (`q`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Attack {
    pub data: Vec<[f64; 3]>,
}

impl Attack {
    pub fn from_columns(alpha: &[f64], beta: &[f64], q: &[f64]) -> Self {
        let n = alpha.len().max(beta.len()).max(q.len());
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            data.push([
                alpha[i.min(alpha.len() - 1)],
                beta[i.min(beta.len() - 1)],
                q[i.min(q.len() - 1)],
            ]);
        }
        Self { data }
    }

    pub fn alpha(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[0]).collect()
    }

    pub fn beta(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[1]).collect()
    }

    pub fn q(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[2]).collect()
    }
}

/// Relative airflow (`aspd`), angle of attack, sideslip, and dynamic pressure (`flow`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Flow {
    pub time: Time,
    pub aspd: Point,
    pub flow: Attack,
}

impl Flow {
    pub fn from_state(body: &State, env: &Environment) -> Self {
        let airspeed = &body.vel - &body.att.inverse().transform_point(&env.wind);
        let n = airspeed.len();
        let mut alpha = Vec::with_capacity(n);
        for &[ax, _, az] in &airspeed.data {
            let a = (az / ax).atan();
            alpha.push(if a.is_finite() { a } else { 0.0 });
        }

        let zeros = vec![0.0; n];
        let stab_airspeed = euler(&Point::from_columns(&zeros, &alpha, &zeros))
            .transform_point(&airspeed);

        let mut beta = Vec::with_capacity(n);
        for &[sx, sy, _] in &stab_airspeed.data {
            let b = (sy / sx).atan();
            beta.push(if b.is_finite() { b } else { 0.0 });
        }

        let rho = env.atm.rho();
        let norms = airspeed.norms();
        let mut q = Vec::with_capacity(n);
        for i in 0..n {
            let qi = 0.5 * rho[i.min(rho.len() - 1)] * norms[i] * norms[i];
            q.push(if qi.is_finite() { qi } else { 0.0 });
        }

        Self {
            time: body.time.clone(),
            aspd: airspeed,
            flow: Attack::from_columns(&alpha, &beta, &q),
        }
    }

    pub fn alpha(&self) -> Vec<f64> {
        self.flow.alpha()
    }

    pub fn beta(&self) -> Vec<f64> {
        self.flow.beta()
    }

    pub fn q(&self) -> Vec<f64> {
        self.flow.q()
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[allow(non_snake_case)]
pub struct Constants {
    pub M: Mass,
    pub S: f64,
    pub b: f64,
    pub c: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Coefficients {
    pub time: Time,
    pub force: Point,
    pub moment: Point,
}

impl Coefficients {
    pub fn from_state(sec: &State, q: &[f64], consts: &Constants) -> Self {
        let u = &sec.vel;
        let du = &sec.acc;
        let w = &sec.rvel;
        let m0 = consts.M.m0();
        let qs: Vec<f64> = q.iter().map(|&qi| (qi * consts.S).max(1e-12)).collect();
        let force = (&(du + &w.cross(u)) * m0).div_scalars(&qs);
        let moment = p0(sec.len());
        Self {
            time: sec.time.clone(),
            force,
            moment,
        }
    }
}
