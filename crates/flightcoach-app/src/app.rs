use egui::{Color32, Pos2, Rect, Sense, Shape, Stroke, Vec2};
use pfc_fcscore::{score_flight_state, score_manoeuvre, ManoeuvreScoreReport};
use pfc_flightdata::{Environment, Flight, Flow, Origin, State};
use pfc_plotting::{boxtrace, get_colour_rgb, plot_regions, plotsec, PlotScene3d, OBJ};
use pfc_schemas::{AJson, FCSResult, ScoreProperties, FCJ};
use std::f64::consts::PI;
use std::path::Path;

pub const SAMPLE_FCJ_JSON: &str =
    include_str!("../../../packages/Schemas/tests/data/fc_json.json");
pub const SAMPLE_BIN_LOG: &[u8] =
    include_bytes!("../../../packages/ArdupilotLogReader/tests/test_inputs/test_log_00000052.BIN");

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActiveTab {
    Viewport3D,
    TelemetryCharts,
    ScoreSheet,
}

#[derive(Debug, Clone)]
pub struct ScoreRow {
    pub index: usize,
    pub name: String,
    pub k: f64,
    pub intra: f64,
    pub inter: f64,
    pub positioning: f64,
    pub total: f64,
    pub weighted: f64,
}

pub struct FlightCoachGui {
    pub loaded_title: String,
    pub full_state: State,
    pub active_state: State,
    pub flow: Flow,
    pub manoeuvres: Vec<String>,
    pub selected_manoeuvre: Option<usize>,
    pub score_rows: Vec<ScoreRow>,
    pub fa_version: String,
    pub schedule_name: String,
    pub score_difficulty: i32,
    pub score_truncate: bool,
    pub fcs_results: Vec<FCSResult>,
    pub man_k_factors: Vec<f64>,
    pub detailed_reports: Vec<ManoeuvreScoreReport>,
    pub aircraft_obj: OBJ,

    // 3D Display options
    pub show_f3a_box: bool,
    pub show_ribbon: bool,
    pub show_wingtips: bool,
    pub show_cg_trace: bool,
    pub show_ghost_models: bool,
    pub ghost_count: usize,
    pub model_scale: f64,
    pub color_by_manoeuvre: bool,
    pub ortho_projection: bool,

    // 3D Camera state (spherical orbit around target in Box frame: +X Right, +Y Out, +Z Up)
    pub cam_azimuth: f64,
    pub cam_elevation: f64,
    pub cam_distance: f64,
    pub cam_target: [f64; 3],

    // Playback state
    pub playing: bool,
    pub cursor_idx: usize,
    pub playback_speed: f64,
    pub active_tab: ActiveTab,
    pub status_message: String,
}

impl Default for FlightCoachGui {
    fn default() -> Self {
        let mut gui = Self {
            loaded_title: String::new(),
            full_state: State::from_constructs(
                pfc_geometry::Time::from_t(&[0.0, 1.0]),
                None,
                None,
                None,
                None,
                None,
            ),
            active_state: State::from_constructs(
                pfc_geometry::Time::from_t(&[0.0, 1.0]),
                None,
                None,
                None,
                None,
                None,
            ),
            flow: Flow::from_state(
                &State::from_constructs(
                    pfc_geometry::Time::from_t(&[0.0, 1.0]),
                    None,
                    None,
                    None,
                    None,
                    None,
                ),
                &Environment::zero(&[0.0, 1.0]),
            ),
            manoeuvres: Vec::new(),
            selected_manoeuvre: None,
            score_rows: Vec::new(),
            fa_version: String::new(),
            schedule_name: "P25".to_string(),
            score_difficulty: 3,
            score_truncate: false,
            fcs_results: Vec::new(),
            man_k_factors: Vec::new(),
            detailed_reports: Vec::new(),
            aircraft_obj: OBJ::default_f3a_plane(),
            show_f3a_box: true,
            show_ribbon: true,
            show_wingtips: true,
            show_cg_trace: true,
            show_ghost_models: true,
            ghost_count: 8,
            model_scale: 4.5,
            color_by_manoeuvre: true,
            ortho_projection: false,
            cam_azimuth: -PI / 2.0,
            cam_elevation: 0.18,
            cam_distance: 420.0,
            cam_target: [0.0, 150.0, 110.0],
            playing: false,
            cursor_idx: 0,
            playback_speed: 1.0,
            active_tab: ActiveTab::Viewport3D,
            status_message: String::new(),
        };
        let _ = gui.load_fcj_str(SAMPLE_FCJ_JSON, "Sample F3A P25 Competition Flight (00000109)");
        // Default to first competition manoeuvre (index 1: trgle) for an immediate crisp view
        if gui.manoeuvres.len() > 1 {
            gui.select_manoeuvre(Some(1));
        }
        gui
    }
}

impl FlightCoachGui {
    pub fn rebuild_score_rows(&mut self) {
        let props = ScoreProperties {
            difficulty: self.score_difficulty,
            truncate: self.score_truncate,
        };
        let mut rows = Vec::new();
        if let Some(latest) = self.fcs_results.last() {
            self.fa_version = latest.fa_version.clone();
            for (idx, opt_mr) in latest.manresults.iter().enumerate() {
                if let Some(mr) = opt_mr {
                    if let Some(sv) = mr.get_score(Some(props)) {
                        let name = self
                            .manoeuvres
                            .get(idx)
                            .cloned()
                            .unwrap_or_else(|| format!("Man {}", idx));
                        let k = self
                            .man_k_factors
                            .get(idx)
                            .copied()
                            .unwrap_or_else(|| {
                                pfc_fcscore::lookup_manoeuvre_def(&self.schedule_name, idx, &name)
                                    .k_factor
                            });
                        if k > 0.0 {
                            rows.push(ScoreRow {
                                index: idx,
                                name,
                                k,
                                intra: sv.intra,
                                inter: sv.inter,
                                positioning: sv.positioning,
                                total: sv.total,
                                weighted: sv.total * k,
                            });
                        }
                    }
                }
            }
        }
        self.score_rows = rows;
    }

    /// Run the native Rust `pfc-fcscore` grading engine on `self.full_state`.
    pub fn run_fcscore_analysis(&mut self) {
        let report = score_flight_state(&mut self.full_state, &self.schedule_name);
        self.manoeuvres = self.full_state.manoeuvre_names();
        self.man_k_factors = self
            .manoeuvres
            .iter()
            .enumerate()
            .map(|(idx, _mname)| {
                report
                    .manoeuvres
                    .iter()
                    .find(|r| r.manoeuvre_index == idx)
                    .map(|r| r.k_factor)
                    .unwrap_or(0.0)
            })
            .collect();
        self.detailed_reports = report.manoeuvres;
        self.fcs_results = vec![report.fcs_result];
        self.rebuild_score_rows();
        self.status_message = format!(
            "Scored {} manoeuvres with {} (Difficulty {}, Truncate={})",
            self.score_rows.len(),
            self.fa_version,
            self.score_difficulty,
            self.score_truncate
        );
    }

    pub fn load_fcj_str(&mut self, json_str: &str, label: &str) -> Result<(), String> {
        let fcj: FCJ = serde_json::from_str(json_str).map_err(|e| e.to_string())?;
        let st = State::from_fc_json(&fcj)?;
        let env = Environment::zero(&st.t());
        let flow = Flow::from_state(&st, &env);
        let mans = st.manoeuvre_names();

        if let Some(sched_name) = fcj.parameters.schedule.get(1) {
            self.schedule_name = sched_name.clone();
        }
        self.man_k_factors = fcj.mans.iter().map(|m| m.k.unwrap_or(0.0)).collect();
        self.fcs_results = fcj.fcs_scores.clone();

        // Populate detailed element reports using preset `els` if available
        let mut detailed = Vec::new();
        let latest_mr = fcj.fcs_scores.last().map(|r| &r.manresults);
        for (idx, m) in fcj.mans.iter().enumerate() {
            let k = m.k.unwrap_or(0.0);
            if k <= 0.0 {
                continue;
            }
            if let Some(sliced) = st.slice_manoeuvre(&m.name) {
                let preset_els = latest_mr
                    .and_then(|mrs| mrs.get(idx))
                    .and_then(|opt| opt.as_ref())
                    .map(|mr| mr.els.as_slice());
                detailed.push(score_manoeuvre(
                    &sliced,
                    &self.schedule_name,
                    idx,
                    &m.name,
                    Some(k),
                    preset_els,
                ));
            }
        }
        self.detailed_reports = detailed;

        self.loaded_title = label.to_string();
        self.full_state = st.clone();
        self.active_state = st;
        self.flow = flow;
        self.manoeuvres = mans;
        self.selected_manoeuvre = None;
        self.cursor_idx = 0;

        if self.fcs_results.is_empty() {
            self.run_fcscore_analysis();
        } else {
            self.rebuild_score_rows();
        }

        self.fit_camera_to_active_state();
        self.status_message = format!(
            "Loaded {} ({} samples, {:.1}s, {} manoeuvres)",
            label,
            self.full_state.len(),
            self.full_state.duration(),
            self.manoeuvres.len()
        );
        Ok(())
    }

    pub fn load_bin_bytes(&mut self, bytes: &[u8], label: &str) -> Result<(), String> {
        let req_types = [
            "ATT", "POS", "GPS", "XKF1", "XKF2", "NKF1", "NKF2", "IMU", "ORGN", "BARO", "ARSP",
            "RCIN", "RCOU", "MODE", "PARM",
        ];
        let log = ardupilot_log_reader::Ardupilot::parse_bytes(
            bytes,
            label.to_string(),
            Some(&req_types),
            None,
            false,
        )?;
        let flight = Flight::from_ardupilot(&log, "pos", 0)?;
        let st = State::from_flight(&flight, None)?;
        let env = Environment::from_flight(&flight, None);
        let flow = Flow::from_state(&st, &env);

        self.loaded_title = label.to_string();
        self.full_state = st.clone();
        self.active_state = st;
        self.flow = flow;
        self.selected_manoeuvre = None;
        self.cursor_idx = 0;
        self.run_fcscore_analysis();
        self.fit_camera_to_active_state();
        self.status_message = format!(
            "Loaded & Scored ArduPilot BIN {} ({} samples, {:.1}s, {} manoeuvres)",
            label,
            self.full_state.len(),
            self.full_state.duration(),
            self.score_rows.len()
        );
        Ok(())
    }

    pub fn load_file_path(&mut self, path: &Path) -> Result<(), String> {
        let ext = path
            .extension()
            .and_then(|s| s.to_str())
            .unwrap_or("")
            .to_ascii_lowercase();
        let fname = path
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("file");

        match ext.as_str() {
            "bin" => {
                let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
                self.load_bin_bytes(&bytes, fname)
            }
            "ajson" => {
                let ajson = AJson::from_file(path)?;
                let st = State::from_ajson(&ajson)?;
                let env = Environment::zero(&st.t());
                self.flow = Flow::from_state(&st, &env);
                self.manoeuvres = st.manoeuvre_names();
                self.full_state = st.clone();
                self.active_state = st;
                self.selected_manoeuvre = None;
                self.loaded_title = fname.to_string();
                self.cursor_idx = 0;
                self.run_fcscore_analysis();
                self.fit_camera_to_active_state();
                self.status_message = format!("Loaded & Scored AJSON {}", fname);
                Ok(())
            }
            "f3a" => {
                let org = Origin::from_f3a_zone(path)?;
                self.status_message = format!(
                    "Applied F3A Zone Pro box '{}' (heading {:.1}°)",
                    org.name,
                    org.heading.to_degrees()
                );
                Ok(())
            }
            _ => {
                let content = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
                if let Ok(()) = self.load_fcj_str(&content, fname) {
                    return Ok(());
                }
                let flight = Flight::from_json_str(&content)?;
                let st = State::from_flight(&flight, None)?;
                let env = Environment::from_flight(&flight, None);
                self.flow = Flow::from_state(&st, &env);
                self.full_state = st.clone();
                self.active_state = st;
                self.selected_manoeuvre = None;
                self.loaded_title = fname.to_string();
                self.cursor_idx = 0;
                self.run_fcscore_analysis();
                self.fit_camera_to_active_state();
                self.status_message = format!("Loaded & Scored Flight JSON {}", fname);
                Ok(())
            }
        }
    }

    pub fn select_manoeuvre(&mut self, idx: Option<usize>) {
        self.selected_manoeuvre = idx;
        if let Some(i) = idx {
            if let Some(mname) = self.manoeuvres.get(i) {
                if let Some(sliced) = self.full_state.slice_manoeuvre(mname) {
                    let env = Environment::zero(&sliced.t());
                    self.flow = Flow::from_state(&sliced, &env);
                    self.active_state = sliced;
                    self.cursor_idx = 0;
                    self.fit_camera_to_active_state();
                    return;
                }
            }
        }
        let env = Environment::zero(&self.full_state.t());
        self.flow = Flow::from_state(&self.full_state, &env);
        self.active_state = self.full_state.clone();
        self.cursor_idx = 0;
        self.fit_camera_to_active_state();
    }

    pub fn fit_camera_to_active_state(&mut self) {
        if self.active_state.is_empty() {
            return;
        }
        let xs = self.active_state.pos.x();
        let ys = self.active_state.pos.y();
        let zs = self.active_state.pos.z();
        let min_x = xs.iter().copied().fold(f64::INFINITY, f64::min);
        let max_x = xs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let min_y = ys.iter().copied().fold(f64::INFINITY, f64::min);
        let max_y = ys.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let min_z = zs.iter().copied().fold(f64::INFINITY, f64::min);
        let max_z = zs.iter().copied().fold(f64::NEG_INFINITY, f64::max);

        self.cam_target = [
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        ];
        let span = (max_x - min_x)
            .max(max_y - min_y)
            .max(max_z - min_z)
            .max(60.0);
        self.cam_distance = span * 1.45;
    }

    pub fn build_current_scene(&self) -> PlotScene3d {
        let mut scene = if self.color_by_manoeuvre && self.selected_manoeuvre.is_none() && !self.manoeuvres.is_empty() {
            let mut s = plot_regions(&self.active_state, "manoeuvre", 0.5 * 1.85 * self.model_scale);
            if self.show_f3a_box {
                let (bmesh, blines) = boxtrace();
                s.add_mesh(bmesh);
                for l in blines {
                    s.add_line(l);
                }
            }
            s
        } else {
            let col = self
                .selected_manoeuvre
                .map(get_colour_rgb)
                .unwrap_or([59, 130, 246]);
            plotsec(
                &self.active_state,
                if self.show_ghost_models {
                    self.ghost_count
                } else {
                    0
                },
                self.model_scale,
                self.show_ribbon,
                self.show_wingtips,
                self.show_cg_trace,
                self.show_f3a_box,
                col,
            )
        };

        // Always render the active aircraft at `cursor_idx` in bright gold/amber
        if !self.active_state.is_empty() {
            let idx = self.cursor_idx.min(self.active_state.len() - 1);
            let tr = self.active_state.row(idx).transform();
            let active_plane = self
                .aircraft_obj
                .scale(self.model_scale * 1.25)
                .transform(&tr)
                .create_mesh([250, 204, 21], 1.0, "Active Aircraft");
            scene.add_mesh(active_plane);
        }
        scene
    }

    pub fn render_ui(&mut self, ctx: &egui::Context) {
        if self.playing && !self.active_state.is_empty() {
            let step = (self.playback_speed.ceil() as usize).max(1);
            self.cursor_idx = (self.cursor_idx + step) % self.active_state.len();
            ctx.request_repaint();
        }

        // Top Toolbar
        egui::TopBottomPanel::top("top_bar").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.heading("✈ FlightCoach Rust");
                ui.separator();

                #[cfg(not(any(target_os = "android", target_os = "ios")))]
                if ui.button("📂 Open File (.BIN / .json / .ajson / .f3a)").clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("FlightCoach & ArduPilot", &["bin", "BIN", "json", "fcj", "ajson", "f3a"])
                        .pick_file()
                    {
                        if let Err(e) = self.load_file_path(&path) {
                            self.status_message = format!("Error: {}", e);
                        }
                    }
                }

                if ui.button("🎯 Sample P25 Flight (FCJ)").clicked() {
                    let _ = self.load_fcj_str(
                        SAMPLE_FCJ_JSON,
                        "Sample F3A P25 Competition Flight (00000109)",
                    );
                }
                if ui.button("🛰 Sample ArduPilot (.BIN)").clicked() {
                    let _ = self.load_bin_bytes(SAMPLE_BIN_LOG, "test_log_00000052.BIN");
                }

                #[cfg(not(any(target_os = "android", target_os = "ios")))]
                if ui.button("🌐 Export 3D Plotly HTML").clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("HTML", &["html"])
                        .set_file_name("flightcoach_plot.html")
                        .save_file()
                    {
                        let html = self.build_current_scene().to_plotly_html(&self.loaded_title);
                        match std::fs::write(&path, html) {
                            Ok(()) => {
                                self.status_message =
                                    format!("Exported Plotly HTML to {}", path.display())
                            }
                            Err(e) => self.status_message = format!("Export failed: {}", e),
                        }
                    }
                }

                ui.separator();
                ui.selectable_value(&mut self.active_tab, ActiveTab::Viewport3D, "🧊 3D Aerobatic Box");
                ui.selectable_value(
                    &mut self.active_tab,
                    ActiveTab::TelemetryCharts,
                    "📈 Telemetry & Kinematics",
                );
                ui.selectable_value(
                    &mut self.active_tab,
                    ActiveTab::ScoreSheet,
                    "🏆 F3A Score Sheet",
                );
            });
        });

        // Bottom Timeline & Live Telemetry Readout
        egui::TopBottomPanel::bottom("bottom_timeline").show(ctx, |ui| {
            let n = self.active_state.len();
            if n > 0 {
                let idx = self.cursor_idx.min(n - 1);
                let t = self.active_state.time.data[idx][0];
                let pos = self.active_state.pos.data[idx];
                let eul = self.active_state.att.row(idx).to_euler().degrees().data[0];
                let speed = self.active_state.vel.row(idx).norm0();
                let g_load = -self.active_state.acc.data[idx][2] / 9.81;

                ui.horizontal(|ui| {
                    let play_label = if self.playing { "⏸ Pause" } else { "▶ Play" };
                    if ui.button(play_label).clicked() {
                        self.playing = !self.playing;
                    }
                    if ui.button("⏮").clicked() {
                        self.cursor_idx = 0;
                    }
                    ui.label("Speed:");
                    for spd in [0.5, 1.0, 2.0, 4.0] {
                        ui.selectable_value(&mut self.playback_speed, spd, format!("{}x", spd));
                    }
                    ui.separator();
                    let mut c_idx = self.cursor_idx;
                    let slider = egui::Slider::new(&mut c_idx, 0..=n.saturating_sub(1))
                        .text(format!("t = {:.2}s", t));
                    if ui.add(slider).changed() {
                        self.cursor_idx = c_idx;
                    }
                });

                ui.horizontal_wrapped(|ui| {
                    ui.label(format!(
                        "Pos (Box): X={:+.1}m  Y={:+.1}m (Depth)  Z={:+.1}m (Alt)",
                        pos[0], pos[1], pos[2]
                    ));
                    ui.separator();
                    ui.label(format!(
                        "Attitude: Roll={:+.1}°  Pitch={:+.1}°  Yaw={:+.1}°",
                        eul[0], eul[1], eul[2]
                    ));
                    ui.separator();
                    ui.label(format!("Speed: {:.1} m/s ({:.0} km/h)", speed, speed * 3.6));
                    ui.separator();
                    ui.label(format!("Normal G: {:+.2}G", g_load));
                    if !self.status_message.is_empty() {
                        ui.separator();
                        ui.colored_label(Color32::from_rgb(125, 211, 252), &self.status_message);
                    }
                });
            }
        });

        // Left Sidebar: Manoeuvre Splitter & 3D Display Controls
        egui::SidePanel::left("left_sidebar")
            .resizable(true)
            .default_width(230.0)
            .show(ctx, |ui| {
                ui.heading("Display & Camera");
                ui.horizontal_wrapped(|ui| {
                    if ui.button("👁 Judge View").clicked() {
                        self.cam_azimuth = -PI / 2.0;
                        self.cam_elevation = 0.14;
                        self.fit_camera_to_active_state();
                    }
                    if ui.button("⬆ Top View").clicked() {
                        self.cam_azimuth = -PI / 2.0;
                        self.cam_elevation = PI / 2.0 - 0.01;
                    }
                    if ui.button("➡ Side View").clicked() {
                        self.cam_azimuth = 0.0;
                        self.cam_elevation = 0.0;
                    }
                });
                ui.checkbox(&mut self.show_f3a_box, "F3A Aerobatic Box Wedge");
                ui.checkbox(&mut self.show_ribbon, "3D Wingtip Ribbon");
                ui.checkbox(&mut self.show_wingtips, "Left/Right Wingtip Trails");
                ui.checkbox(&mut self.show_cg_trace, "CG Flight Trajectory");
                ui.checkbox(&mut self.show_ghost_models, "Sequence Aircraft Models");
                ui.checkbox(&mut self.ortho_projection, "Orthographic Projection");

                ui.add(
                    egui::Slider::new(&mut self.model_scale, 1.0..=15.0).text("Aircraft Scale"),
                );
                if self.show_ghost_models {
                    ui.add(egui::Slider::new(&mut self.ghost_count, 2..=25).text("Model Count"));
                }

                ui.separator();
                ui.heading("Manoeuvres");
                egui::ScrollArea::vertical().show(ui, |ui| {
                    let all_selected = self.selected_manoeuvre.is_none();
                    if ui
                        .selectable_label(
                            all_selected,
                            format!("🌍 Full Flight ({} samples)", self.full_state.len()),
                        )
                        .clicked()
                    {
                        self.select_manoeuvre(None);
                    }
                    let mans_clone = self.manoeuvres.clone();
                    for (i, mname) in mans_clone.iter().enumerate() {
                        let [r, g, b] = get_colour_rgb(i);
                        let score_suffix = self
                            .score_rows
                            .iter()
                            .find(|sr| sr.index == i)
                            .map(|sr| format!(" (K={:.0} | {:.2})", sr.k, sr.total))
                            .unwrap_or_default();
                        ui.horizontal(|ui| {
                            let (rect, _) =
                                ui.allocate_exact_size(Vec2::new(10.0, 10.0), Sense::hover());
                            ui.painter()
                                .rect_filled(rect, 2.0, Color32::from_rgb(r, g, b));
                            if ui
                                .selectable_label(
                                    self.selected_manoeuvre == Some(i),
                                    format!("{}. {}{}", i, mname, score_suffix),
                                )
                                .clicked()
                            {
                                self.select_manoeuvre(Some(i));
                            }
                        });
                    }
                });
            });

        // Central Panel
        egui::CentralPanel::default().show(ctx, |ui| match self.active_tab {
            ActiveTab::Viewport3D => self.draw_3d_viewport(ui),
            ActiveTab::TelemetryCharts => self.draw_telemetry_charts(ui),
            ActiveTab::ScoreSheet => self.draw_score_sheet(ui),
        });
    }

    fn draw_3d_viewport(&mut self, ui: &mut egui::Ui) {
        let rect = ui.available_rect_before_wrap();
        let response = ui.allocate_rect(rect, Sense::click_and_drag());

        // Handle camera orbit, pan, and zoom
        if response.dragged_by(egui::PointerButton::Primary) {
            let delta = response.drag_delta();
            self.cam_azimuth -= (delta.x as f64) * 0.008;
            self.cam_elevation =
                (self.cam_elevation + (delta.y as f64) * 0.008).clamp(-1.52, 1.52);
        } else if response.dragged_by(egui::PointerButton::Secondary)
            || response.dragged_by(egui::PointerButton::Middle)
        {
            let delta = response.drag_delta();
            let scale = self.cam_distance * 0.0015;
            let (sin_az, cos_az) = self.cam_azimuth.sin_cos();
            let right = [-sin_az, cos_az, 0.0];
            self.cam_target[0] -= (delta.x as f64) * scale * right[0];
            self.cam_target[1] -= (delta.x as f64) * scale * right[1];
            self.cam_target[2] += (delta.y as f64) * scale;
        }

        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if response.hovered() && scroll.abs() > 0.1 {
            let zoom_factor = (1.0 - (scroll as f64) * 0.002).clamp(0.8, 1.25);
            self.cam_distance = (self.cam_distance * zoom_factor).clamp(20.0, 2500.0);
        }

        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::from_rgb(15, 23, 42));

        let cam = Camera3D::new(
            self.cam_target,
            self.cam_azimuth,
            self.cam_elevation,
            self.cam_distance,
            self.ortho_projection,
            rect,
        );

        let scene = self.build_current_scene();

        // Collect depth-sorted draw primitives (triangles and line segments)
        enum DrawPrim {
            Triangle {
                depth: f64,
                pts: [Pos2; 3],
                fill: Color32,
            },
            Segment {
                depth: f64,
                p0: Pos2,
                p1: Pos2,
                stroke: Stroke,
            },
        }

        let mut prims: Vec<DrawPrim> = Vec::new();
        let light_dir = normalize3([-0.3, -0.6, 0.75]);

        for mesh in &scene.meshes {
            let [cr, cg, cb] = mesh.color_rgb;
            let alpha = (mesh.opacity.clamp(0.0, 1.0) * 255.0) as u8;
            for &[i0, i1, i2] in &mesh.faces {
                if i0 >= mesh.vertices.len() || i1 >= mesh.vertices.len() || i2 >= mesh.vertices.len() {
                    continue;
                }
                let v0 = mesh.vertices[i0];
                let v1 = mesh.vertices[i1];
                let v2 = mesh.vertices[i2];
                if let (Some((p0, z0)), Some((p1, z1)), Some((p2, z2))) =
                    (cam.project(v0), cam.project(v1), cam.project(v2))
                {
                    let e1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
                    let e2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];
                    let n = normalize3([
                        e1[1] * e2[2] - e1[2] * e2[1],
                        e1[2] * e2[0] - e1[0] * e2[2],
                        e1[0] * e2[1] - e1[1] * e2[0],
                    ]);
                    let ndotl = (n[0] * light_dir[0] + n[1] * light_dir[1] + n[2] * light_dir[2]).abs();
                    let shade = (0.45 + 0.55 * ndotl) as f32;
                    let fill = Color32::from_rgba_unmultiplied(
                        ((cr as f32) * shade).min(255.0) as u8,
                        ((cg as f32) * shade).min(255.0) as u8,
                        ((cb as f32) * shade).min(255.0) as u8,
                        alpha,
                    );
                    prims.push(DrawPrim::Triangle {
                        depth: (z0 + z1 + z2) / 3.0,
                        pts: [p0, p1, p2],
                        fill,
                    });
                }
            }
        }

        for line in &scene.lines {
            if line.points.len() < 2 {
                continue;
            }
            let [cr, cg, cb] = line.color_rgb;
            let stroke = Stroke::new(line.width, Color32::from_rgb(cr, cg, cb));
            for pair in line.points.windows(2) {
                if let (Some((p0, z0)), Some((p1, z1))) =
                    (cam.project(pair[0]), cam.project(pair[1]))
                {
                    prims.push(DrawPrim::Segment {
                        depth: 0.5 * (z0 + z1),
                        p0,
                        p1,
                        stroke,
                    });
                }
            }
        }

        // Painter's algorithm: far to near
        prims.sort_by(|a, b| {
            let da = match a {
                DrawPrim::Triangle { depth, .. } | DrawPrim::Segment { depth, .. } => *depth,
            };
            let db = match b {
                DrawPrim::Triangle { depth, .. } | DrawPrim::Segment { depth, .. } => *depth,
            };
            db.partial_cmp(&da).unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut batch_mesh = egui::Mesh::default();
        for prim in prims {
            match prim {
                DrawPrim::Triangle { pts, fill, .. } => {
                    let base = batch_mesh.vertices.len() as u32;
                    batch_mesh.colored_vertex(pts[0], fill);
                    batch_mesh.colored_vertex(pts[1], fill);
                    batch_mesh.colored_vertex(pts[2], fill);
                    batch_mesh.add_triangle(base, base + 1, base + 2);
                }
                DrawPrim::Segment { p0, p1, stroke, .. } => {
                    if !batch_mesh.is_empty() {
                        painter.add(Shape::mesh(std::mem::take(&mut batch_mesh)));
                    }
                    painter.line_segment([p0, p1], stroke);
                }
            }
        }
        if !batch_mesh.is_empty() {
            painter.add(Shape::mesh(batch_mesh));
        }

        // Overlay HUD caption
        painter.text(
            rect.left_top() + Vec2::new(12.0, 10.0),
            egui::Align2::LEFT_TOP,
            format!(
                "{} — Drag: Orbit | Right-Drag: Pan | Scroll: Zoom",
                self.loaded_title
            ),
            egui::FontId::proportional(14.0),
            Color32::from_rgb(226, 232, 240),
        );
    }

    fn draw_telemetry_charts(&mut self, ui: &mut egui::Ui) {
        let n = self.active_state.len();
        if n < 2 {
            ui.label("Not enough samples to plot telemetry.");
            return;
        }
        let ts = self.active_state.t();
        let eul = self.active_state.att.to_euler().degrees();
        let roll = eul.x();
        let pitch = eul.y();
        let yaw = eul.z();
        let speeds = self.active_state.vel.norms();
        let alts = self.active_state.pos.z();
        let g_loads: Vec<f64> = self
            .active_state
            .acc
            .data
            .iter()
            .map(|r| -r[2] / 9.81)
            .collect();
        let alphas: Vec<f64> = self.flow.alpha().into_iter().map(|a| a.to_degrees()).collect();
        let betas: Vec<f64> = self.flow.beta().into_iter().map(|b| b.to_degrees()).collect();

        let cursor_t = ts[self.cursor_idx.min(n - 1)];

        egui::ScrollArea::vertical().show(ui, |ui| {
            draw_mini_chart(
                ui,
                "Attitude Euler Angles (deg) — Roll (Red), Pitch (Green), Yaw (Blue)",
                &ts,
                &[
                    (&roll, Color32::from_rgb(239, 68, 68)),
                    (&pitch, Color32::from_rgb(34, 197, 94)),
                    (&yaw, Color32::from_rgb(59, 130, 246)),
                ],
                cursor_t,
                160.0,
            );
            ui.add_space(8.0);
            draw_mini_chart(
                ui,
                "Box Altitude Z (m, Cyan) & Ground Speed (m/s, Amber)",
                &ts,
                &[
                    (&alts, Color32::from_rgb(56, 189, 248)),
                    (&speeds, Color32::from_rgb(251, 191, 36)),
                ],
                cursor_t,
                160.0,
            );
            ui.add_space(8.0);
            draw_mini_chart(
                ui,
                "Normal Load Factor (G, Violet) & Angle of Attack Alpha / Sideslip Beta (deg)",
                &ts,
                &[
                    (&g_loads, Color32::from_rgb(167, 139, 250)),
                    (&alphas, Color32::from_rgb(248, 113, 113)),
                    (&betas, Color32::from_rgb(96, 165, 250)),
                ],
                cursor_t,
                160.0,
            );
        });
    }

    fn draw_score_sheet(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.heading(format!(
                "F3A Competition Score Sheet — {} (FA Version: {})",
                self.loaded_title,
                if self.fa_version.is_empty() {
                    "N/A"
                } else {
                    &self.fa_version
                }
            ));
            ui.separator();
            ui.label("Schedule:");
            for sched in ["P25", "P23"] {
                if ui
                    .selectable_label(self.schedule_name == sched, sched)
                    .clicked()
                {
                    self.schedule_name = sched.to_string();
                }
            }
            ui.separator();
            ui.label("Difficulty:");
            for (d, label) in [(1, "1 (Easy)"), (2, "2 (Med)"), (3, "3 (Hard)")] {
                if ui
                    .selectable_label(self.score_difficulty == d, label)
                    .clicked()
                {
                    self.score_difficulty = d;
                    self.rebuild_score_rows();
                }
            }
            ui.separator();
            if ui
                .checkbox(&mut self.score_truncate, "Truncate (0.5 pt step)")
                .changed()
            {
                self.rebuild_score_rows();
            }
            ui.separator();
            if ui
                .button("⚡ Score Flight (Rust FCScore)")
                .on_hover_text("Segment elements and compute Intra, Inter, and Positioning downgrades offline")
                .clicked()
            {
                self.run_fcscore_analysis();
            }
        });
        ui.separator();

        if self.score_rows.is_empty() {
            ui.label("No score rows present. Click '⚡ Score Flight (Rust FCScore)' to grade this log.");
            return;
        }

        let total_k: f64 = self.score_rows.iter().map(|r| r.k).sum();
        let total_weighted: f64 = self.score_rows.iter().map(|r| r.weighted).sum();
        let normalized_1000 = if total_k > 0.0 {
            total_weighted / (total_k * 10.0) * 1000.0
        } else {
            0.0
        };

        ui.horizontal(|ui| {
            ui.label(format!("Total K-Factor: {:.0}", total_k));
            ui.separator();
            ui.label(format!(
                "Raw Weighted Total: {:.2} / {:.0}",
                total_weighted,
                total_k * 10.0
            ));
            ui.separator();
            ui.colored_label(
                Color32::from_rgb(52, 211, 153),
                format!("Normalized Score: {:.2} / 1000", normalized_1000),
            );
        });
        ui.separator();

        egui::ScrollArea::vertical().show(ui, |ui| {
            egui::Grid::new("f3a_score_grid")
                .striped(true)
                .min_col_width(80.0)
                .show(ui, |ui| {
                    ui.strong("#");
                    ui.strong("Manoeuvre");
                    ui.strong("K-Factor");
                    ui.strong("Intra DG");
                    ui.strong("Inter DG");
                    ui.strong("Positioning DG");
                    ui.strong("Score (/10)");
                    ui.strong("Weighted (K×Score)");
                    ui.strong("Actions");
                    ui.end_row();

                    let rows = self.score_rows.clone();
                    for row in rows {
                        ui.label(format!("{}", row.index));
                        ui.label(&row.name);
                        ui.label(format!("{:.0}", row.k));
                        ui.label(format!("-{:.2}", row.intra));
                        ui.label(format!("-{:.2}", row.inter));
                        ui.label(format!("-{:.2}", row.positioning));
                        ui.colored_label(
                            Color32::from_rgb(125, 211, 252),
                            format!("{:.2}", row.total),
                        );
                        ui.strong(format!("{:.2}", row.weighted));
                        ui.horizontal(|ui| {
                            if ui.button("🔬 Elements").clicked() {
                                self.selected_manoeuvre = Some(row.index);
                            }
                            if ui.button("🔍 3D").clicked() {
                                self.select_manoeuvre(Some(row.index));
                                self.active_tab = ActiveTab::Viewport3D;
                            }
                        });
                        ui.end_row();
                    }
                });

            // Detailed Element & Downgrade Inspector for the selected manoeuvre
            let inspect_idx = self.selected_manoeuvre.unwrap_or(1);
            if let Some(rep) = self
                .detailed_reports
                .iter()
                .find(|r| r.manoeuvre_index == inspect_idx)
                .cloned()
            {
                ui.add_space(12.0);
                ui.separator();
                ui.heading(format!(
                    "Element & Downgrade Breakdown — #{}. {} ({}, K={:.0})",
                    rep.manoeuvre_index, rep.short_name, rep.full_name, rep.k_factor
                ));
                ui.horizontal_wrapped(|ui| {
                    ui.label(format!(
                        "Positioning: Center Offset={:.1}° (DG -{:.2}) | Max Box Angle Lat={:.1}° Vert={:.1}° (DG -{:.2}) | Mean Depth Y={:.1}m (DG -{:.2})",
                        rep.positioning.center_offset_deg,
                        rep.positioning.centering_downgrade,
                        rep.positioning.max_lateral_angle_deg,
                        rep.positioning.max_vertical_angle_deg,
                        rep.positioning.box_boundary_downgrade,
                        rep.positioning.mean_depth_y_m,
                        rep.positioning.depth_downgrade,
                    ));
                });
                if !rep.inter.items.is_empty() {
                    ui.horizontal_wrapped(|ui| {
                        ui.strong("Inter-Element Downgrades:");
                        for it in &rep.inter.items {
                            ui.label(format!(
                                "{} ({:.1}% err -> -{:.2})",
                                it.description, it.relative_error_pct, it.raw_downgrade
                            ));
                            ui.separator();
                        }
                    });
                }
                ui.add_space(4.0);
                egui::Grid::new("element_breakdown_grid")
                    .striped(true)
                    .min_col_width(75.0)
                    .show(ui, |ui| {
                        ui.strong("Element");
                        ui.strong("Kind");
                        ui.strong("Samples");
                        ui.strong("Duration");
                        ui.strong("Mean Speed");
                        ui.strong("Length / Radius");
                        ui.strong("Intra Downgrade Breakdown");
                        ui.strong("Raw Intra DG");
                        ui.end_row();

                        for el in &rep.elements {
                            ui.label(&el.element_name);
                            ui.label(el.kind.as_str());
                            ui.label(format!("{}..{}", el.start_idx, el.stop_idx));
                            ui.label(format!("{:.2}s", el.duration_s));
                            ui.label(format!("{:.1} m/s", el.mean_speed_mps));
                            if let Some(r) = el.mean_radius_m {
                                ui.label(format!("R = {:.1}m", r));
                            } else {
                                ui.label(format!("L = {:.1}m", el.line_length_m));
                            }
                            let summary: Vec<String> = el
                                .items
                                .iter()
                                .map(|it| {
                                    format!(
                                        "{}: {:.1} (vis {:.2}) -> -{:.2}",
                                        it.name,
                                        it.peak_error_deg_or_pct,
                                        it.mean_visibility,
                                        it.raw_downgrade
                                    )
                                })
                                .collect();
                            ui.label(summary.join(" | "));
                            ui.colored_label(
                                Color32::from_rgb(248, 113, 113),
                                format!("-{:.2}", el.raw_intra_total),
                            );
                            ui.end_row();
                        }
                    });
            }
        });
    }
}

struct Camera3D {
    eye: [f64; 3],
    forward: [f64; 3],
    right: [f64; 3],
    up: [f64; 3],
    distance: f64,
    ortho: bool,
    rect: Rect,
}

impl Camera3D {
    fn new(
        target: [f64; 3],
        azimuth: f64,
        elevation: f64,
        distance: f64,
        ortho: bool,
        rect: Rect,
    ) -> Self {
        let (sin_el, cos_el) = elevation.sin_cos();
        let (sin_az, cos_az) = azimuth.sin_cos();
        let offset = [
            distance * cos_el * cos_az,
            distance * cos_el * sin_az,
            distance * sin_el,
        ];
        let eye = [
            target[0] + offset[0],
            target[1] + offset[1],
            target[2] + offset[2],
        ];
        let forward = normalize3([target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]]);
        let world_up = [0.0, 0.0, 1.0];
        let right = normalize3(cross3(forward, world_up));
        let up = normalize3(cross3(right, forward));
        Self {
            eye,
            forward,
            right,
            up,
            distance,
            ortho,
            rect,
        }
    }

    fn project(&self, pt: [f64; 3]) -> Option<(Pos2, f64)> {
        let rel = [pt[0] - self.eye[0], pt[1] - self.eye[1], pt[2] - self.eye[2]];
        let z_cam = dot3(rel, self.forward);
        if !self.ortho && z_cam <= 1.0 {
            return None;
        }
        let x_cam = dot3(rel, self.right);
        let y_cam = dot3(rel, self.up);
        let half_h = 0.5 * (self.rect.height() as f64);
        let scale = if self.ortho {
            half_h / (0.6 * self.distance)
        } else {
            (half_h * 1.35) / z_cam
        };
        let cx = self.rect.center().x as f64;
        let cy = self.rect.center().y as f64;
        let sx = (cx + x_cam * scale) as f32;
        let sy = (cy - y_cam * scale) as f32;
        Some((Pos2::new(sx, sy), z_cam))
    }
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalize3(v: [f64; 3]) -> [f64; 3] {
    let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if n <= 1e-12 {
        [1.0, 0.0, 0.0]
    } else {
        [v[0] / n, v[1] / n, v[2] / n]
    }
}

fn draw_mini_chart(
    ui: &mut egui::Ui,
    title: &str,
    ts: &[f64],
    series: &[(&[f64], Color32)],
    cursor_t: f64,
    height: f32,
) {
    ui.label(egui::RichText::new(title).strong());
    let (rect, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), height), Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 4.0, Color32::from_rgb(15, 23, 42));

    let t0 = *ts.first().unwrap_or(&0.0);
    let t1 = (*ts.last().unwrap_or(&1.0)).max(t0 + 1e-6);

    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for (vals, _) in series {
        for &v in *vals {
            if v.is_finite() {
                min_y = min_y.min(v);
                max_y = max_y.max(v);
            }
        }
    }
    if !min_y.is_finite() || (max_y - min_y).abs() < 1e-6 {
        min_y = -1.0;
        max_y = 1.0;
    }

    let map_pt = |t: f64, y: f64| -> Pos2 {
        let fx = ((t - t0) / (t1 - t0)).clamp(0.0, 1.0) as f32;
        let fy = ((y - min_y) / (max_y - min_y)).clamp(0.0, 1.0) as f32;
        Pos2::new(
            rect.left() + fx * rect.width(),
            rect.bottom() - 8.0 - fy * (rect.height() - 16.0),
        )
    };

    // Zero line if in range
    if min_y <= 0.0 && max_y >= 0.0 {
        let p_left = map_pt(t0, 0.0);
        let p_right = map_pt(t1, 0.0);
        painter.line_segment(
            [p_left, p_right],
            Stroke::new(1.0_f32, Color32::from_rgb(51, 65, 85)),
        );
    }

    for (vals, color) in series {
        let pts: Vec<Pos2> = ts
            .iter()
            .zip(vals.iter())
            .map(|(&t, &y)| map_pt(t, y))
            .collect();
        if pts.len() >= 2 {
            painter.add(Shape::line(pts, Stroke::new(1.5_f32, *color)));
        }
    }

    // Cursor line
    let cx = map_pt(cursor_t, min_y).x;
    painter.line_segment(
        [Pos2::new(cx, rect.top()), Pos2::new(cx, rect.bottom())],
        Stroke::new(1.5_f32, Color32::from_rgb(250, 204, 21)),
    );
}

#[cfg(not(any(target_os = "android", target_os = "ios")))]
impl eframe::App for FlightCoachGui {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.render_ui(ctx);
    }
}
