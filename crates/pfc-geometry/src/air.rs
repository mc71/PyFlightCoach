use serde::{Deserialize, Serialize};

pub const R_GAS: f64 = 287.058;
pub const GAMMA: f64 = 1.4;

#[inline]
pub fn get_rho(pressure_pa: f64, temperature_k: f64) -> f64 {
    pressure_pa / (R_GAS * temperature_k)
}

/// Atmospheric state `[(P_pa, T_k, rho_kg_m3); N]`, matching `geometry.Air`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Air {
    pub data: Vec<[f64; 3]>,
}

impl Air {
    pub fn new(p: f64, t: f64, rho: f64) -> Self {
        Self {
            data: vec![[p, t, rho]],
        }
    }

    pub fn from_rows(data: Vec<[f64; 3]>) -> Self {
        Self { data }
    }

    pub fn iso_sea_level(length: usize) -> Self {
        let p = 101_325.0;
        let t = 288.15;
        let rho = get_rho(p, t);
        Self {
            data: vec![[p, t, rho]; length.max(1)],
        }
    }

    pub fn from_pt(pressure: &[f64], temperature: &[f64]) -> Self {
        let n = pressure.len().max(temperature.len());
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            let p = pressure[i.min(pressure.len() - 1)];
            let t = temperature[i.min(temperature.len() - 1)];
            data.push([p, t, get_rho(p, t)]);
        }
        Self { data }
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.data.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    pub fn p(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[0]).collect()
    }

    pub fn t(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[1]).collect()
    }

    pub fn rho(&self) -> Vec<f64> {
        self.data.iter().map(|r| r[2]).collect()
    }
}
