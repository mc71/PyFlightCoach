use crate::environment::{Environment, Flow};
use crate::flight::Flight;
use crate::labels::{Label, LabelGroup, LabelGroups};
use crate::origin::Origin;
use pfc_geometry::{
    euler, get_index, p0, px, py, pz, q0, Coord, IndexDirection, Mass, Point, Quaternion, Time,
    Transformation,
};
use pfc_schemas::ajson::AJson;
use pfc_schemas::fcj::{Data as FcjData, Man as FcjMan, Parameters as FcjParameters, View as FcjView, FCJ};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::f64::consts::PI;

/// Cubic Hermite / kinematic trajectory interpolation between `(t0, p0, v0)` and `(t1, p1, v1)`.
pub fn kinematic_interp_pva(
    t0: f64,
    t1: f64,
    p0: &Point,
    p1: &Point,
    v0: &Point,
    v1: &Point,
    t: f64,
) -> (Point, Point, Point) {
    let dt1 = t1 - t0;
    let denom = (dt1.powi(3) / 6.0) - (dt1.powi(2) / 4.0);
    let m = &(p1 - p0 - &(v0 * dt1) - &(&(v1 - v0) * 0.5)) / denom;
    let b = &(&(v1 - v0) / (dt1 * dt1)) - &(&m * 0.5);
    let dt = t - t0;
    let a = &b + &(&m * dt);
    let v = v0 + &(&b * dt) + &(&m * (0.5 * dt * dt));
    let p = p0 + &(v0 * dt) + &(&b * (0.5 * dt * dt)) + &(&m * (dt.powi(3) / 6.0));
    (p, v, a)
}

/// 6-DOF Aircraft State in Aerobatic Box / Judge Frame (`pos`, `att` in Box frame; `vel`, `rvel`, `acc` in Body frame).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct State {
    pub time: Time,
    pub pos: Point,
    pub att: Quaternion,
    pub vel: Point,
    pub rvel: Point,
    pub acc: Point,
    #[serde(default)]
    pub labels: LabelGroups,
}

impl State {
    pub const CONSTRUCT_FREQ: f64 = 25.0;

    pub fn from_constructs(
        time: Time,
        pos: Option<Point>,
        att: Option<Quaternion>,
        vel: Option<Point>,
        rvel: Option<Point>,
        acc: Option<Point>,
    ) -> Self {
        let n = time.len().max(1);
        let dt = time.dt();
        let pos = pos.unwrap_or_else(|| p0(n));
        let att = att.unwrap_or_else(|| q0(n));
        let vel = vel.unwrap_or_else(|| {
            if n <= 1 {
                p0(n)
            } else {
                att.inverse().transform_point(&pos.diff(&dt, "gradient"))
            }
        });
        let rvel = rvel.unwrap_or_else(|| {
            if n <= 1 {
                p0(n)
            } else {
                att.body_diff(&dt)
            }
        });
        let acc = acc.unwrap_or_else(|| {
            if n <= 1 {
                p0(n)
            } else {
                let wvel = att.transform_point(&vel);
                let wacc_plus_g = &wvel.diff(&dt, "gradient") + &pz(9.81, n);
                att.inverse().transform_point(&wacc_plus_g)
            }
        });

        Self {
            time,
            pos,
            att,
            vel,
            rvel,
            acc,
            labels: LabelGroups::default(),
        }
    }

    pub fn with_labels(mut self, labels: LabelGroups) -> Self {
        self.labels = labels;
        self
    }

    pub fn with_single_label(mut self, group: &str, label_name: &str) -> Self {
        if !self.time.is_empty() {
            let t0 = self.time.t0();
            let t1 = self.time.data.last().unwrap()[0];
            let mut lg = LabelGroup::new();
            lg.labels.insert(label_name.to_string(), Label::new(t0, t1));
            self.labels.insert(group, lg);
        }
        self
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.time.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.time.is_empty()
    }

    pub fn t(&self) -> Vec<f64> {
        self.time.t()
    }

    pub fn dt(&self) -> Vec<f64> {
        self.time.dt()
    }

    pub fn duration(&self) -> f64 {
        self.time.duration()
    }

    pub fn transform(&self) -> Transformation {
        Transformation::build(&self.pos, &self.att)
    }

    pub fn back_transform(&self) -> Transformation {
        Transformation::build(&(-&self.pos), &self.att.inverse())
    }

    pub fn from_transform(transform: &Transformation, time: Option<Time>) -> Self {
        let n = transform.len().max(1);
        let time = time.unwrap_or_else(|| {
            let dur = Self::CONSTRUCT_FREQ * (n as f64);
            let ts: Vec<f64> = if n <= 1 {
                vec![0.0]
            } else {
                (0..n)
                    .map(|i| dur * (i as f64) / ((n - 1) as f64))
                    .collect()
            };
            Time::from_t(&ts)
        });
        Self::from_constructs(
            time,
            Some(transform.p.clone()),
            Some(transform.q.clone()),
            None,
            None,
            None,
        )
    }

    pub fn row(&self, idx: usize) -> Self {
        let t = self.time.row(idx);
        let t_val = t.t0();
        Self {
            time: t,
            pos: self.pos.row(idx),
            att: self.att.row(idx),
            vel: self.vel.row(idx),
            rvel: self.rvel.row(idx),
            acc: self.acc.row(idx),
            labels: self.labels.slice(t_val, t_val),
        }
    }

    pub fn slice_iloc(&self, start: usize, stop: usize) -> Self {
        let e = stop.min(self.len());
        let s = start.min(e);
        if s >= e {
            return Self::from_constructs(Time::from_t(&[0.0]), None, None, None, None, None);
        }
        let time = self.time.slice(s, e);
        let t0 = time.t0();
        let t1 = time.data.last().unwrap()[0];
        Self {
            time,
            pos: self.pos.slice(s, e),
            att: self.att.slice(s, e),
            vel: self.vel.slice(s, e),
            rvel: self.rvel.slice(s, e),
            acc: self.acc.slice(s, e),
            labels: self.labels.slice(t0, t1),
        }
    }

    pub fn slice_time(&self, t_start: f64, t_stop: f64) -> Self {
        let ts = self.t();
        if ts.is_empty() {
            return self.clone();
        }
        let s_idx = ts
            .iter()
            .position(|&t| t >= t_start - 1e-9)
            .unwrap_or(0);
        let e_idx = ts
            .iter()
            .rposition(|&t| t <= t_stop + 1e-9)
            .map(|i| i + 1)
            .unwrap_or(ts.len());
        self.slice_iloc(s_idx, e_idx.max(s_idx + 1))
    }

    pub fn slice_manoeuvre(&self, name: &str) -> Option<Self> {
        let lg = self.labels.get("manoeuvre")?;
        let lab = lg.get(name)?;
        let mut sliced = self.slice_time(lab.start, lab.stop);
        if !lab.sublabels.is_empty() {
            for (k, v) in &lab.sublabels.groups {
                sliced.labels.insert(k.clone(), v.clone());
            }
        }
        Some(sliced)
    }

    pub fn slice_element(&self, name: &str) -> Option<Self> {
        let lg = self.labels.get("element")?;
        let lab = lg.get(name)?;
        Some(self.slice_time(lab.start, lab.stop))
    }

    pub fn manoeuvre_names(&self) -> Vec<String> {
        self.labels
            .get("manoeuvre")
            .map(|lg| lg.labels.keys().cloned().collect())
            .unwrap_or_default()
    }

    pub fn body_to_world(&self, pin: &Point, rotation_only: bool) -> Point {
        if rotation_only {
            self.transform().rotate_point(pin)
        } else {
            self.transform().point(pin)
        }
    }

    pub fn world_to_body(&self, pin: &Point, rotation_only: bool) -> Point {
        if rotation_only {
            self.back_transform().rotate_point(pin)
        } else {
            self.back_transform().point(pin)
        }
    }

    pub fn wvel(&self) -> Point {
        self.att.transform_point(&self.vel)
    }

    pub fn wacc(&self) -> Point {
        self.att.transform_point(&self.acc)
    }

    pub fn wrvel(&self) -> Point {
        self.att.transform_point(&self.rvel)
    }

    /// Construct a `State` from a `Flight` and an optional `Origin`.
    pub fn from_flight(flight: &Flight, origin: Option<&Origin>) -> Result<Self, String> {
        let org = origin.unwrap_or(&flight.origin);
        let rot = org.rotation();

        let pos = if flight.primary_pos_source.starts_with("pos")
            || flight.primary_pos_source.starts_with("gps")
        {
            if let Some(gps) = flight.pos_gps() {
                rot.transform_point(&(&gps - &org.pos))
            } else if let Some(p_ned) = flight.position_ned() {
                rot.transform_point(&p_ned)
            } else {
                return Err("Flight missing position coordinates".to_string());
            }
        } else if let Some(p_ned) = flight.position_ned() {
            rot.transform_point(&p_ned)
        } else if let Some(gps) = flight.pos_gps() {
            rot.transform_point(&(&gps - &org.pos))
        } else {
            return Err("Flight missing position coordinates".to_string());
        };

        let att_eul = flight
            .attitude_euler()
            .ok_or_else(|| "Flight missing attitude".to_string())?;
        let att = &rot * &euler(&att_eul);

        let vel = flight
            .velocity_ned()
            .map(|v_ned| att.inverse().transform_point(&rot.transform_point(&v_ned)));
        let rvel = flight.axisrate();
        let acc = flight.acceleration();

        Ok(Self::from_constructs(
            Time::from_t(flight.time_flight()),
            Some(pos),
            Some(att),
            vel,
            rvel,
            acc,
        ))
    }

    /// Construct a `State` directly from a FlightCoach JSON (`FCJ`), including manoeuvre labels.
    pub fn from_fc_json(fcj: &FCJ) -> Result<Self, String> {
        let flight = Flight::from_fc_json(fcj);
        let state = Self::from_flight(&flight, None)?;
        if !fcj.mans.is_empty() {
            Ok(state.splitter_labels(&fcj.mans, None, "manoeuvre", 0.0))
        } else {
            Ok(state)
        }
    }

    /// Construct a `State` from an `AJson` manoeuvre or full `AJson`.
    pub fn from_json_records(records: &[serde_json::Value]) -> Result<Self, String> {
        if records.is_empty() {
            return Err("Empty JSON records for State".to_string());
        }
        let n = records.len();
        let mut t = Vec::with_capacity(n);
        let mut dt = Vec::with_capacity(n);
        let mut x = Vec::with_capacity(n);
        let mut y = Vec::with_capacity(n);
        let mut z = Vec::with_capacity(n);
        let mut rw = Vec::with_capacity(n);
        let mut rx = Vec::with_capacity(n);
        let mut ry = Vec::with_capacity(n);
        let mut rz = Vec::with_capacity(n);
        let mut u = Vec::with_capacity(n);
        let mut v = Vec::with_capacity(n);
        let mut w = Vec::with_capacity(n);
        let mut p = Vec::with_capacity(n);
        let mut q = Vec::with_capacity(n);
        let mut r = Vec::with_capacity(n);
        let mut du = Vec::with_capacity(n);
        let mut dv = Vec::with_capacity(n);
        let mut dw = Vec::with_capacity(n);
        let mut man_labels = Vec::new();
        let mut el_labels = Vec::new();

        let get_f = |obj: &serde_json::Value, key: &str, def: f64| -> f64 {
            obj.get(key).and_then(|v| v.as_f64()).unwrap_or(def)
        };

        for rec in records {
            t.push(get_f(rec, "t", 0.0));
            dt.push(get_f(rec, "dt", 0.04));
            x.push(get_f(rec, "x", 0.0));
            y.push(get_f(rec, "y", 0.0));
            z.push(get_f(rec, "z", 0.0));
            rw.push(get_f(rec, "rw", 1.0));
            rx.push(get_f(rec, "rx", 0.0));
            ry.push(get_f(rec, "ry", 0.0));
            rz.push(get_f(rec, "rz", 0.0));
            u.push(get_f(rec, "u", 0.0));
            v.push(get_f(rec, "v", 0.0));
            w.push(get_f(rec, "w", 0.0));
            p.push(get_f(rec, "p", 0.0));
            q.push(get_f(rec, "q", 0.0));
            r.push(get_f(rec, "r", 0.0));
            du.push(get_f(rec, "du", 0.0));
            dv.push(get_f(rec, "dv", 0.0));
            dw.push(get_f(rec, "dw", 0.0));
            if let Some(m) = rec.get("manoeuvre").and_then(|v| v.as_str()) {
                man_labels.push(m.to_string());
            }
            if let Some(e) = rec.get("element").and_then(|v| v.as_str()) {
                el_labels.push(e.to_string());
            }
        }

        let time_rows: Vec<[f64; 2]> = t.iter().copied().zip(dt.iter().copied()).map(|(a, b)| [a, b]).collect();
        let mut state = Self::from_constructs(
            Time::from_rows(time_rows),
            Some(Point::from_columns(&x, &y, &z)),
            Some(Quaternion::from_columns(&rw, &rx, &ry, &rz)),
            Some(Point::from_columns(&u, &v, &w)),
            Some(Point::from_columns(&p, &q, &r)),
            Some(Point::from_columns(&du, &dv, &dw)),
        );
        if man_labels.len() == n {
            state
                .labels
                .insert("manoeuvre", LabelGroup::read_array(&t, &man_labels));
        }
        if el_labels.len() == n {
            state
                .labels
                .insert("element", LabelGroup::read_array(&t, &el_labels));
        }
        Ok(state)
    }

    pub fn from_ajson(ajson: &AJson) -> Result<Self, String> {
        let mut states = Vec::new();
        for m in &ajson.mans {
            if let Some(arr) = m.flown.as_array() {
                if let Ok(st) = Self::from_json_records(arr) {
                    states.push(st.with_single_label("manoeuvre", &m.name));
                }
            }
        }
        if states.is_empty() {
            return Err("No valid flown manoeuvre states in AJson".to_string());
        }
        Ok(Self::concatenate(&states))
    }

    pub fn concatenate(sts: &[Self]) -> Self {
        let mut time_t = Vec::new();
        let mut pos_rows = Vec::new();
        let mut att_rows = Vec::new();
        let mut vel_rows = Vec::new();
        let mut rvel_rows = Vec::new();
        let mut acc_rows = Vec::new();
        let mut labels = LabelGroups::new();

        for st in sts {
            time_t.extend_from_slice(&st.t());
            pos_rows.extend_from_slice(&st.pos.data);
            att_rows.extend_from_slice(&st.att.data);
            vel_rows.extend_from_slice(&st.vel.data);
            rvel_rows.extend_from_slice(&st.rvel.data);
            acc_rows.extend_from_slice(&st.acc.data);
            for (gname, lg) in &st.labels.groups {
                let entry = labels
                    .groups
                    .entry(gname.clone())
                    .or_insert_with(LabelGroup::new);
                for (lname, lab) in &lg.labels {
                    let mut unique = lname.clone();
                    let mut c = 2usize;
                    while entry.labels.contains_key(&unique) {
                        unique = format!("{}{}", lname, c);
                        c += 1;
                    }
                    entry.labels.insert(unique, lab.clone());
                }
            }
        }

        Self::from_constructs(
            Time::from_t(&time_t),
            Some(Point::from_rows(pos_rows)),
            Some(Quaternion::from_rows(att_rows)),
            Some(Point::from_rows(vel_rows)),
            Some(Point::from_rows(rvel_rows)),
            Some(Point::from_rows(acc_rows)),
        )
        .with_labels(labels)
    }

    pub fn stack(sts: &[Self], overlap: usize) -> Self {
        if sts.is_empty() {
            return Self::from_constructs(Time::from_t(&[0.0]), None, None, None, None, None);
        }
        let mut aligned = vec![sts[0].clone()];
        for st in &sts[1..] {
            let prev = aligned.last().unwrap();
            let prev_end = prev.time.data.last().unwrap()[0];
            let prev_dt = prev.time.data.last().unwrap()[1];
            let cur_start = st.time.t0();
            let offset = if overlap >= 1 {
                prev_end - cur_start
            } else {
                prev_end + prev_dt - cur_start
            };
            let mut shifted = st.clone();
            shifted.time = &st.time + offset;
            shifted.labels = st.labels.offset(offset);
            if overlap >= 1 && shifted.len() > 1 {
                let n = shifted.len();
                let mut trimmed = shifted.slice_iloc(1, n);
                trimmed.labels = shifted.labels;
                aligned.push(trimmed);
            } else {
                aligned.push(shifted);
            }
        }
        Self::concatenate(&aligned)
    }

    /// Label the manoeuvres in a `State` based on the FlightCoach splitter `mans` array.
    pub fn splitter_labels(
        &self,
        mans: &[FcjMan],
        better_names: Option<&[String]>,
        target_col: &str,
        t0: f64,
    ) -> Self {
        if mans.is_empty() || self.is_empty() {
            return self.clone();
        }
        let ts = self.t();
        let i0 = get_index(&ts, t0, IndexDirection::Forward, None)
            .map(|v| v.round() as usize)
            .unwrap_or(0);

        let mut lg = LabelGroup::new();
        let mut used_names = Vec::new();

        for (i, split_man) in mans.iter().enumerate() {
            let mut base_name = if i == 0 {
                split_man.name.clone()
            } else if let Some(bnames) = better_names {
                if i - 1 < bnames.len() {
                    bnames[i - 1].clone()
                } else {
                    "land".to_string()
                }
            } else {
                split_man.name.clone()
            };
            while used_names.contains(&base_name) {
                base_name.push('2');
            }
            used_names.push(base_name.clone());

            let start_idx = if i == 0 {
                0
            } else {
                ((split_man.start.max(0) as usize) + i0).min(ts.len() - 1)
            };
            let stop_idx = ((split_man.stop.max(0) as usize) + i0).min(ts.len() - 1);
            if start_idx <= stop_idx {
                lg.labels
                    .insert(base_name, Label::new(ts[start_idx], ts[stop_idx]));
            }
        }

        let mut out = self.clone();
        out.labels.insert(target_col, lg);
        out
    }

    /// Project forward through `time` assuming uniform circular motion.
    pub fn fill(&self, time: &Time) -> Self {
        let last = self.row(self.len() - 1);
        let n = time.len();
        let t_rel: Vec<f64> = time.t().into_iter().map(|ti| ti - time.t0()).collect();

        let vel = last.vel.tile(n);
        let rvel_single = if last.vel.norm0() > 1e-9 {
            last.rvel.vector_rejection(&last.vel)
        } else {
            last.rvel.clone()
        };
        let rvel = rvel_single.tile(n);
        let att = last.att.body_rotate(&rvel.mul_scalars(&t_rel));
        let wvel0 = last.att.transform_point(&last.vel);
        let wrvel0 = last.att.transform_point(&rvel_single);

        let (pos, acc_world) = if rvel_single.norm0() > 1e-9 && last.vel.norm0() > 1e-9 {
            let theta = wrvel0.tile(n).mul_scalars(&t_rel);
            let denom = wrvel0.vector_rejection(&wvel0).norm0().max(1e-12);
            let r0 = wrvel0.cross(&wvel0).unit() * (wvel0.norm0() / denom);
            let center = &last.pos + &r0;
            let radius = Quaternion::from_axis_angle(&theta).transform_point(&(-&r0));
            let speed_sq = wvel0.norm0().powi(2);
            let r_norms = radius.norms();
            let centripetal = (-&radius.unit()).mul_scalars(
                &r_norms
                    .into_iter()
                    .map(|rn| speed_sq / rn.max(1e-12))
                    .collect::<Vec<_>>(),
            );
            let acc_w = &centripetal + &pz(9.81, n);
            let p = &center.tile(n) + &radius;
            (p, acc_w)
        } else {
            let p = &last.pos.tile(n) + &wvel0.tile(n).mul_scalars(&t_rel);
            let acc_w = pz(9.81, n);
            (p, acc_w)
        };

        Self::from_constructs(
            time.clone(),
            Some(pos),
            Some(att.clone()),
            Some(vel),
            Some(rvel),
            Some(att.inverse().transform_point(&acc_world)),
        )
    }

    pub fn extrapolate(&self, duration: f64, min_len: usize) -> Self {
        let dt0 = self.time.dt0().max(1e-4);
        let npoints = ((duration / dt0).ceil() as usize).max(min_len);
        let ts: Vec<f64> = (0..npoints)
            .map(|i| duration * (i as f64) / ((npoints - 1).max(1) as f64))
            .collect();
        self.fill(&Time::from_t(&ts))
    }

    pub fn body_rotate(&self, r: &Point) -> Self {
        let att = self.att.body_rotate(r);
        let q = &att.inverse() * &self.att;
        let mut rvel = q.transform_point(&self.rvel);
        if r.len() == self.len() && self.len() > 1 {
            let rdiff = Point::concatenate(&[p0(1), r.diff(&self.dt(), "diff")]);
            rvel = &rvel + &rdiff;
        }
        Self::from_constructs(
            self.time.clone(),
            Some(self.pos.clone()),
            Some(att),
            Some(q.transform_point(&self.vel)),
            Some(rvel),
            Some(q.transform_point(&self.acc)),
        )
        .with_labels(self.labels.clone())
    }

    pub fn scale(&self, factor: f64) -> Self {
        Self::from_constructs(
            self.time.clone(),
            Some(&self.pos * factor),
            Some(self.att.clone()),
            Some(&self.vel * factor),
            Some(self.rvel.clone()),
            Some(&self.acc * factor),
        )
        .with_labels(self.labels.clone())
    }

    pub fn mirror_zy(&self) -> Self {
        let eul = self.att.to_euler();
        let shifted = &eul + &Point::new(0.0, 0.0, PI);
        let mirrored_eul = &shifted * &Point::new(-1.0, 1.0, -1.0);
        let att = Quaternion::from_euler(&mirrored_eul);
        Self::from_constructs(
            self.time.clone(),
            Some(&self.pos * &Point::new(-1.0, 1.0, 1.0)),
            Some(att),
            Some(self.vel.clone()),
            None,
            None,
        )
        .with_labels(self.labels.clone())
    }

    pub fn body_to_stability(&self, flow: Option<&Flow>) -> Self {
        let owned_flow;
        let fl = match flow {
            Some(f) => f,
            None => {
                let env = Environment::from_constructs(self.time.clone(), None, None);
                owned_flow = Flow::from_state(self, &env);
                &owned_flow
            }
        };
        let rot = (-py(1.0, self.len())).mul_scalars(&fl.alpha());
        self.body_rotate(&rot)
    }

    pub fn stability_to_wind(&self, flow: Option<&Flow>) -> Self {
        let owned_flow;
        let fl = match flow {
            Some(f) => f,
            None => {
                let env = Environment::from_constructs(self.time.clone(), None, None);
                owned_flow = Flow::from_state(self, &env);
                &owned_flow
            }
        };
        let rot = pz(1.0, self.len()).mul_scalars(&fl.beta());
        self.body_rotate(&rot)
    }

    pub fn body_to_wind(&self, flow: Option<&Flow>) -> Self {
        self.body_to_stability(flow).stability_to_wind(flow)
    }

    pub fn to_track(&self) -> Self {
        self.body_to_wind(None)
    }

    pub fn wind_to_body(&self, flow: &Flow) -> Self {
        let stab = self.body_rotate(&(-pz(1.0, self.len())).mul_scalars(&flow.beta()));
        stab.body_rotate(&py(1.0, self.len()).mul_scalars(&flow.alpha()))
    }

    pub fn track_to_wind(&self, env: &Environment) -> Self {
        let jwind = self.att.inverse().transform_point(&env.wind);
        let yaw_rot = &(&jwind + &self.vel).angles(&px(1.0, 1)) * &Point::new(0.0, 0.0, 1.0);
        let int_axis = self.body_rotate(&yaw_rot);
        let intwind = int_axis.att.inverse().transform_point(&env.wind);
        let pitch_rot = &(&intwind + &int_axis.vel).angles(&px(1.0, 1)) * &Point::new(0.0, 1.0, 0.0);
        int_axis.body_rotate(&pitch_rot)
    }

    pub fn direction(&self) -> Vec<f64> {
        self.att
            .transform_point(&px(1.0, 1))
            .data
            .iter()
            .map(|r| r[0].signum())
            .collect()
    }

    pub fn cross_direction(&self) -> Vec<f64> {
        self.att
            .transform_point(&px(1.0, 1))
            .data
            .iter()
            .map(|r| r[1].signum())
            .collect()
    }

    pub fn inverted(&self) -> Vec<bool> {
        self.att.is_inverted()
    }

    pub fn upright(&self) -> Vec<bool> {
        self.inverted().into_iter().map(|b| !b).collect()
    }

    pub fn judging_itrans(&self, template_itrans: &Transformation) -> Transformation {
        let c = Coord::from_xy(
            &p0(1),
            &self.att.row(0).transform_point(&self.vel.row(0)),
            &template_itrans.att().transform_point(&py(1.0, 1)),
        );
        Transformation::build(
            &self.pos.row(0),
            &Quaternion::from_rotation_matrix(&c.rotation_matrix()).inverse(),
        )
    }

    pub fn move_state(&self, transform: &Transformation) -> Self {
        Self::from_constructs(
            self.time.clone(),
            Some(transform.point(&self.pos)),
            Some(transform.rotate_quat(&self.att)),
            Some(self.vel.clone()),
            Some(self.rvel.clone()),
            Some(self.acc.clone()),
        )
        .with_labels(self.labels.clone())
    }

    pub fn move_back(&self, transform: &Transformation) -> Self {
        let s1 = self.move_state(&Transformation::build(&(-&transform.p), &q0(1)));
        s1.move_state(&Transformation::build(&p0(1), &transform.q.inverse()))
    }

    pub fn relocate(&self, start_pos: &Point) -> Self {
        let offset = start_pos - &self.pos.row(0);
        self.move_state(&Transformation::build(&offset, &q0(1)))
    }

    pub fn superimpose_angles(&self, angles: &Point, reference: &str) -> Self {
        if reference == "body" {
            let rot = Quaternion::from_axis_angle(angles).inverse();
            Self::from_constructs(
                self.time.clone(),
                Some(self.pos.clone()),
                Some(self.att.body_rotate(angles)),
                Some(rot.transform_point(&self.vel)),
                Some(&rot.transform_point(&self.rvel) + &angles.diff(&self.dt(), "gradient")),
                Some(rot.transform_point(&self.acc)),
            )
            .with_labels(self.labels.clone())
        } else {
            let att = self.att.rotate(angles);
            let att_inv = att.inverse();
            Self::from_constructs(
                self.time.clone(),
                Some(self.pos.clone()),
                Some(att),
                Some(att_inv.transform_point(&self.att.transform_point(&self.vel))),
                Some(
                    att_inv.transform_point(
                        &(&self.att.transform_point(&self.rvel) + &angles.diff(&self.dt(), "gradient")),
                    ),
                ),
                Some(att_inv.transform_point(&self.att.transform_point(&self.acc))),
            )
            .with_labels(self.labels.clone())
        }
    }

    pub fn superimpose_rotation(&self, axis: &Point, angle: f64, reference: &str) -> Self {
        let dur = if self.duration() > 0.0 {
            self.duration()
        } else {
            self.time.dt0()
        };
        let t0 = self.time.t0();
        let scalars: Vec<f64> = self.t().into_iter().map(|ti| (ti - t0) * angle / dur).collect();
        let angles = axis.unit().tile(self.len()).mul_scalars(&scalars);
        self.superimpose_angles(&angles, reference)
    }

    pub fn superimpose_roll(&self, angle: f64) -> Self {
        self.superimpose_rotation(&px(1.0, 1), angle, "body")
    }

    pub fn zero_g_acc(&self) -> Point {
        &self.att.inverse().transform_point(&pz(-9.81, 1)) + &self.acc
    }

    pub fn arc_centre(&self) -> Point {
        let acc = self.zero_g_acc().vector_rejection(&self.vel);
        let v_norms = self.vel.norms();
        let a_norms = acc.norms();
        let scalars: Vec<f64> = v_norms
            .iter()
            .zip(a_norms.iter())
            .map(|(&v, &a)| if a > 1e-12 { (v * v) / a } else { 0.0 })
            .collect();
        acc.unit().mul_scalars(&scalars)
    }

    pub fn curvature(&self, axis: &Point) -> Vec<f64> {
        let trfl = self.to_track();
        let u = trfl.vel.x();
        let u_sq: Vec<f64> = u.into_iter().map(|v| (v * v).max(1e-12)).collect();
        let body_curv = (-&trfl.zero_g_acc()).div_scalars(&u_sq).cross(&px(1.0, 1));
        let world_curv = trfl.att.transform_point(&body_curv);
        world_curv.scalar_projection(axis)
    }

    pub fn f_gravity(&self, mass: &Mass) -> Point {
        self.att
            .inverse()
            .transform_point(&pz(-9.81, 1))
            .mul_scalars(&mass.m())
    }

    pub fn f_inertia(&self, mass: &Mass) -> Point {
        (&self.zero_g_acc() + &self.rvel.cross(&self.vel)).mul_scalars(&mass.m())
    }

    pub fn m_inertia(&self, mass: &Mass) -> Point {
        let h = mass.angular_momentum(&self.rvel);
        &h.diff(&self.dt(), "gradient") + &self.rvel.cross(&h)
    }

    pub fn get_rotation(&self) -> Vec<f64> {
        let sp = self.rvel.scalar_projection(&self.vel);
        let dt = self.dt();
        let mut acc = 0.0;
        sp.into_iter()
            .zip(dt.into_iter())
            .map(|(s, d)| {
                acc += s * d;
                acc
            })
            .collect()
    }

    pub fn create_fc_json(
        &self,
        kfactors: &[f64],
        schedule_name: &str,
        schedule_category: &str,
        scores: Option<Vec<Option<f64>>>,
    ) -> FCJ {
        let wvels = self.transform().rotate_point(&self.vel);
        let tr = Transformation::from_coords(
            &Coord::from_xy(&p0(1), &px(1.0, 1), &py(1.0, 1)),
            &Coord::from_xy(&p0(1), &px(1.0, 1), &py(-1.0, 1)),
        );
        let eul = tr.rotate_quat(&self.att).to_euler();
        let ts = self.t();

        let mut data = Vec::with_capacity(self.len());
        for i in 0..self.len() {
            let [x, y, z] = self.pos.data[i];
            let [vx, vy, vz] = wvels.data[i];
            let [rx, ry, rz] = eul.data[i];
            data.push(FcjData {
                VN: Some(vx),
                VE: Some(-vy),
                VD: Some(-vz),
                dPD: None,
                r: rx.to_degrees(),
                p: ry.to_degrees(),
                yw: rz.to_degrees(),
                N: x,
                E: -y,
                D: -z,
                time: (ts[i] * 1e6) as i64,
                roll: rx,
                pitch: ry,
                yaw: rz,
                wN: Some(0.0),
                wE: Some(0.0),
            });
        }

        let mut mans = Vec::new();
        if let Some(lg) = self.labels.get("manoeuvre") {
            let dur = self.duration().max(1e-9);
            for (i, (k, label)) in lg.labels.iter().enumerate() {
                if let Ok(lab_iloc) = label.to_iloc(&ts) {
                    mans.push(FcjMan {
                        name: k.clone(),
                        k: kfactors.get(i).copied(),
                        id: format!("sp_{}", i),
                        sp: i as i64,
                        wd: 100.0 * label.width() / dur,
                        start: lab_iloc.start.round() as i64,
                        stop: lab_iloc.stop.round() as i64,
                        sel: false,
                        background: String::new(),
                    });
                }
            }
        }

        let (start_idx, stop_idx) = mans
            .get(1)
            .map(|m| (m.start, m.stop))
            .unwrap_or((0, self.len().saturating_sub(1) as i64));

        let mut pos_map = HashMap::new();
        pos_map.insert("x".to_string(), serde_json::json!(-120.0));
        pos_map.insert("y".to_string(), serde_json::json!(130.5));
        pos_map.insert("z".to_string(), serde_json::json!(265.0));

        let mut tgt_map = HashMap::new();
        tgt_map.insert("x".to_string(), serde_json::json!(-22.0));
        tgt_map.insert("y".to_string(), serde_json::json!(160.0));
        tgt_map.insert("z".to_string(), serde_json::json!(-204.0));

        let scored = scores.is_some();
        let default_scores = vec![
            Some(0.0),
            Some(10.0),
            Some(10.0),
            Some(10.0),
            Some(10.0),
            Some(10.0),
            Some(10.0),
            Some(10.0),
            Some(10.0),
            Some(10.0),
            Some(10.0),
            Some(10.0),
            Some(10.0),
            Some(10.0),
            Some(10.0),
            Some(10.0),
            Some(10.0),
            Some(10.0),
            Some(600.0),
        ];

        FCJ {
            version: "1.3".to_string(),
            comments: "DO NOT EDIT\n".to_string(),
            name: schedule_name.to_string(),
            view: FcjView {
                position: pos_map,
                target: tgt_map,
            },
            parameters: FcjParameters {
                rotation: -PI / 2.0,
                start: start_idx,
                stop: stop_idx,
                moveEast: 0.0,
                moveNorth: 0.0,
                wingspan: 3.0,
                modelwingspan: 25.0,
                elevate: 0.0,
                originLat: 0.0,
                originLng: 0.0,
                originAlt: 0.0,
                pilotLat: 0.0,
                pilotLng: 0.0,
                pilotAlt: 0.0,
                centerLat: 0.0,
                centerLng: -0.1,
                centerAlt: 0.0,
                schedule: vec![schedule_category.to_string(), schedule_name.to_string()],
            },
            scored,
            scores: scores.unwrap_or(default_scores),
            human_scores: Vec::new(),
            fcs_scores: Vec::new(),
            mans,
            data,
            jhash: None,
        }
    }
}
