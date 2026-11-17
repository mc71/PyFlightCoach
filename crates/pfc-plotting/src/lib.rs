use pfc_flightdata::State;
use pfc_geometry::{euler_angles, p0, py, Coord, Point, Transformation};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

pub const COLD_DRAFT_F3A_PLANE_OBJ: &str =
    include_str!("../../../packages/Plotting/src/plotting/data/ColdDraftF3APlane.obj");

pub const PLOTLY_QUALITATIVE_COLORS: &[(&str, [u8; 3])] = &[
    ("#636EFA", [99, 110, 250]),
    ("#EF553B", [239, 85, 59]),
    ("#00CC96", [0, 204, 150]),
    ("#AB63FA", [171, 99, 250]),
    ("#FFA15A", [255, 161, 90]),
    ("#19D3F3", [25, 211, 243]),
    ("#FF6692", [255, 102, 146]),
    ("#B6E880", [182, 232, 128]),
    ("#FF97FF", [255, 151, 255]),
    ("#FECB52", [254, 203, 82]),
];

pub fn get_colour_hex(i: usize) -> &'static str {
    PLOTLY_QUALITATIVE_COLORS[i % PLOTLY_QUALITATIVE_COLORS.len()].0
}

pub fn get_colour_rgb(i: usize) -> [u8; 3] {
    PLOTLY_QUALITATIVE_COLORS[i % PLOTLY_QUALITATIVE_COLORS.len()].1
}

/// 3D Wavefront OBJ model matching `plotting.model.OBJ`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OBJ {
    pub vertices: Point,
    pub normals: Point,
    pub faces: Vec<[usize; 3]>,
}

impl OBJ {
    pub fn from_obj_data(odata: &str) -> Self {
        let mut verts = Vec::new();
        let mut norms = Vec::new();
        let mut faces = Vec::new();

        for line in odata.lines() {
            let trimmed = line.trim();
            if let Some(rest) = trimmed.strip_prefix("v ") {
                let nums: Vec<f64> = rest
                    .split_whitespace()
                    .filter_map(|s| s.parse::<f64>().ok())
                    .collect();
                if nums.len() >= 3 {
                    verts.push([nums[0], nums[1], nums[2]]);
                }
            } else if let Some(rest) = trimmed.strip_prefix("vn ") {
                let nums: Vec<f64> = rest
                    .split_whitespace()
                    .filter_map(|s| s.parse::<f64>().ok())
                    .collect();
                if nums.len() >= 3 {
                    norms.push([nums[0], nums[1], nums[2]]);
                }
            } else if let Some(rest) = trimmed.strip_prefix("f ") {
                let idx: Vec<usize> = rest
                    .split_whitespace()
                    .filter_map(|tok| {
                        let v_str = tok.split('/').next()?;
                        v_str.parse::<usize>().ok().map(|v| v.saturating_sub(1))
                    })
                    .collect();
                if idx.len() >= 3 {
                    faces.push([idx[0], idx[1], idx[2]]);
                }
            }
        }

        Self {
            vertices: Point::from_rows(verts),
            normals: Point::from_rows(norms),
            faces,
        }
    }

    /// Returns the default aligned F3A aircraft model matching `plotting.model.obj`.
    pub fn default_f3a_plane() -> Self {
        let raw = Self::from_obj_data(COLD_DRAFT_F3A_PLANE_OBJ);
        let align = Transformation::new(
            Point::new(0.75, 0.0, 0.0),
            euler_angles(PI, 0.0, -PI / 2.0),
        );
        raw.transform(&align)
    }

    pub fn scale(&self, scale_factor: f64) -> Self {
        Self {
            vertices: &self.vertices * scale_factor,
            normals: self.normals.clone(),
            faces: self.faces.clone(),
        }
    }

    pub fn transform(&self, transformation: &Transformation) -> Self {
        Self {
            vertices: transformation.point(&self.vertices),
            normals: self.normals.clone(),
            faces: self.faces.clone(),
        }
    }

    pub fn create_mesh(&self, color_rgb: [u8; 3], opacity: f32, name: impl Into<String>) -> Mesh3dTrace {
        Mesh3dTrace {
            name: name.into(),
            vertices: self.vertices.data.clone(),
            faces: self.faces.clone(),
            color_rgb,
            opacity,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Line3dTrace {
    pub name: String,
    pub points: Vec<[f64; 3]>,
    pub color_rgb: [u8; 3],
    pub width: f32,
    pub show_legend: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Mesh3dTrace {
    pub name: String,
    pub vertices: Vec<[f64; 3]>,
    pub faces: Vec<[usize; 3]>,
    pub color_rgb: [u8; 3],
    pub opacity: f32,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PlotScene3d {
    pub lines: Vec<Line3dTrace>,
    pub meshes: Vec<Mesh3dTrace>,
}

impl PlotScene3d {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_line(&mut self, line: Line3dTrace) {
        self.lines.push(line);
    }

    pub fn add_mesh(&mut self, mesh: Mesh3dTrace) {
        self.meshes.push(mesh);
    }

    pub fn merge(&mut self, other: Self) {
        self.lines.extend(other.lines);
        self.meshes.extend(other.meshes);
    }

    /// Export interactive 3D Plotly HTML document (`judge_view` camera).
    pub fn to_plotly_html(&self, title: &str) -> String {
        let mut traces_json = Vec::new();
        for line in &self.lines {
            let xs: Vec<f64> = line.points.iter().map(|p| p[0]).collect();
            let ys: Vec<f64> = line.points.iter().map(|p| p[1]).collect();
            let zs: Vec<f64> = line.points.iter().map(|p| p[2]).collect();
            let [r, g, b] = line.color_rgb;
            traces_json.push(serde_json::json!({
                "type": "scatter3d",
                "mode": "lines",
                "name": line.name,
                "x": xs,
                "y": ys,
                "z": zs,
                "showlegend": line.show_legend,
                "line": {
                    "color": format!("rgb({},{},{})", r, g, b),
                    "width": line.width,
                }
            }));
        }
        for mesh in &self.meshes {
            let xs: Vec<f64> = mesh.vertices.iter().map(|p| p[0]).collect();
            let ys: Vec<f64> = mesh.vertices.iter().map(|p| p[1]).collect();
            let zs: Vec<f64> = mesh.vertices.iter().map(|p| p[2]).collect();
            let is: Vec<usize> = mesh.faces.iter().map(|f| f[0]).collect();
            let js: Vec<usize> = mesh.faces.iter().map(|f| f[1]).collect();
            let ks: Vec<usize> = mesh.faces.iter().map(|f| f[2]).collect();
            let [r, g, b] = mesh.color_rgb;
            traces_json.push(serde_json::json!({
                "type": "mesh3d",
                "name": mesh.name,
                "x": xs,
                "y": ys,
                "z": zs,
                "i": is,
                "j": js,
                "k": ks,
                "color": format!("rgb({},{},{})", r, g, b),
                "opacity": mesh.opacity,
                "showscale": false,
            }));
        }

        let layout = serde_json::json!({
            "title": title,
            "margin": {"l": 0, "r": 0, "t": 30, "b": 0},
            "scene": {
                "aspectmode": "data",
                "camera": {
                    "up": {"x": 0.0, "y": 0.0, "z": 1.0},
                    "center": {"x": 0.0, "y": 0.0, "z": -0.2},
                    "eye": {"x": 0.0, "y": -3.0, "z": -0.8}
                }
            }
        });

        format!(
            r#"<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>{title}</title>
  <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
  <style>html, body, #plot {{ margin: 0; width: 100%; height: 100%; }}</style>
</head>
<body>
  <div id="plot"></div>
  <script>
    const data = {data};
    const layout = {layout};
    Plotly.newPlot('plot', data, layout, {{responsive: true}});
  </script>
</body>
</html>"#,
            title = title,
            data = serde_json::to_string(&traces_json).unwrap_or_else(|_| "[]".to_string()),
            layout = serde_json::to_string(&layout).unwrap_or_else(|_| "{}".to_string()),
        )
    }
}

/// Center of gravity 3D line trace (`traces.cgtrace`).
pub fn cgtrace(seq: &State, name: impl Into<String>, color_rgb: [u8; 3], width: f32) -> Line3dTrace {
    Line3dTrace {
        name: name.into(),
        points: seq.pos.data.clone(),
        color_rgb,
        width,
        show_legend: false,
    }
}

/// Left (blue) and right (red) wingtip 3D line traces (`traces.tiptrace`).
pub fn tiptrace(seq: &State, span: f64) -> [Line3dTrace; 2] {
    let left = seq.body_to_world(&Point::new(0.0, span * 0.5, 0.0), false);
    let right = seq.body_to_world(&Point::new(0.0, -span * 0.5, 0.0), false);
    [
        Line3dTrace {
            name: "Left Tip".to_string(),
            points: left.data,
            color_rgb: [59, 130, 246], // blue
            width: 1.5,
            show_legend: false,
        },
        Line3dTrace {
            name: "Right Tip".to_string(),
            points: right.data,
            color_rgb: [239, 68, 68], // red
            width: 1.5,
            show_legend: false,
        },
    ]
}

/// 3D wingtip ribbon triangle strip mesh (`traces.ribbon`).
pub fn ribbon(
    sec: &State,
    span: f64,
    color_rgb: [u8; 3],
    opacity: f32,
    name: impl Into<String>,
) -> Mesh3dTrace {
    let left = sec.body_to_world(&Point::new(0.0, span * 0.5, 0.0), false);
    let right = sec.body_to_world(&Point::new(0.0, -span * 0.5, 0.0), false);
    let n = sec.len();
    let mut vertices = Vec::with_capacity(2 * n);
    for i in 0..n {
        vertices.push(left.data[i]);
        vertices.push(right.data[i]);
    }
    let mut faces = Vec::with_capacity(2 * n.saturating_sub(1));
    if n >= 2 {
        for s in 0..n - 1 {
            let v0 = 2 * s;
            let v1 = 2 * s + 1;
            let v2 = 2 * s + 2;
            let v3 = 2 * s + 3;
            faces.push([v0, v1, v2]);
            faces.push([v1, v3, v2]);
        }
    }
    Mesh3dTrace {
        name: name.into(),
        vertices,
        faces,
        color_rgb,
        opacity,
    }
}

/// Place `npoints` scaled 3D aircraft meshes along `seq` (`traces.meshes`).
pub fn meshes(
    npoints: usize,
    seq: &State,
    color_rgb: [u8; 3],
    scale: f64,
    obj: Option<&OBJ>,
) -> Vec<Mesh3dTrace> {
    if npoints == 0 || seq.is_empty() {
        return Vec::new();
    }
    let default_obj;
    let base_obj = match obj {
        Some(o) => o,
        None => {
            default_obj = OBJ::default_f3a_plane();
            &default_obj
        }
    };
    let scaled_obj = base_obj.scale(scale);
    let len = seq.len();

    let mut locs = Vec::new();
    if npoints >= 1 {
        locs.push(0usize);
    }
    if npoints >= 3 && len > 2 {
        let step = (len as f64) / ((npoints - 1) as f64);
        for k in 1..=(npoints - 2) {
            let idx = ((k as f64) * step).floor() as usize;
            locs.push(idx.min(len - 1));
        }
    }
    if npoints >= 2 {
        locs.push(len - 1);
    }

    let ts = seq.t();
    locs.into_iter()
        .map(|loc| {
            let tr = seq.row(loc).transform();
            scaled_obj
                .transform(&tr)
                .create_mesh(color_rgb, 1.0, format!("{:.1}s", ts[loc]))
        })
        .collect()
}

/// Coordinate frame axes line traces (`x: red`, `y: blue`, `z: green`) (`traces.axestrace`).
pub fn axestrace(cid: &Coord, length: f64) -> Vec<Line3dTrace> {
    let mut out = Vec::new();
    for i in 0..cid.len() {
        let o = cid.origin.data[i];
        let x = cid.x_axis.data[i];
        let y = cid.y_axis.data[i];
        let z = cid.z_axis.data[i];
        out.push(Line3dTrace {
            name: "X Axis".to_string(),
            points: vec![o, [o[0] + x[0] * length, o[1] + x[1] * length, o[2] + x[2] * length]],
            color_rgb: [239, 68, 68],
            width: 2.0,
            show_legend: false,
        });
        out.push(Line3dTrace {
            name: "Y Axis".to_string(),
            points: vec![o, [o[0] + y[0] * length, o[1] + y[1] * length, o[2] + y[2] * length]],
            color_rgb: [59, 130, 246],
            width: 2.0,
            show_legend: false,
        });
        out.push(Line3dTrace {
            name: "Z Axis".to_string(),
            points: vec![o, [o[0] + z[0] * length, o[1] + z[1] * length, o[2] + z[2] * length]],
            color_rgb: [34, 197, 94],
            width: 2.0,
            show_legend: false,
        });
    }
    out
}

/// F3A aerobatic box wedge mesh (`traces.boxtrace`) plus wireframe boundary lines for crisp rendering.
pub fn boxtrace() -> (Mesh3dTrace, Vec<Line3dTrace>) {
    let xlim = 170.0 * 60.0_f64.to_radians().tan();
    let ylim = 170.0;
    let vertices = vec![
        [0.0, 0.0, 0.0],       // 0: pilot origin
        [xlim, ylim, 0.0],     // 1: bottom right
        [0.0, ylim, 0.0],      // 2: bottom center
        [-xlim, ylim, 0.0],    // 3: bottom left
        [xlim, ylim, xlim],    // 4: top right
        [0.0, ylim, xlim],     // 5: top center
        [-xlim, ylim, xlim],   // 6: top left
    ];
    let faces = vec![
        [0, 1, 2],
        [0, 2, 3],
        [0, 1, 4],
        [0, 3, 6],
        [0, 4, 6],
    ];
    let mesh = Mesh3dTrace {
        name: "F3A Box".to_string(),
        vertices: vertices.clone(),
        faces,
        color_rgb: [148, 163, 184],
        opacity: 0.25,
    };

    let wire_color = [100, 116, 139];
    let edges = [
        (0, 1, "Box Right Bottom"),
        (0, 3, "Box Left Bottom"),
        (0, 4, "Box Right Top"),
        (0, 6, "Box Left Top"),
        (0, 2, "Box Center Floor"),
        (2, 5, "Box Center Line"),
        (3, 1, "Box Far Bottom"),
        (6, 4, "Box Far Top"),
        (1, 4, "Box Far Right"),
        (3, 6, "Box Far Left"),
    ];
    let lines = edges
        .into_iter()
        .map(|(a, b, name)| Line3dTrace {
            name: name.to_string(),
            points: vec![vertices[a], vertices[b]],
            color_rgb: wire_color,
            width: 1.5,
            show_legend: false,
        })
        .collect();

    (mesh, lines)
}

/// High-level 3D section plotter matching `plotting.plots.plotsec`.
#[allow(clippy::too_many_arguments)]
pub fn plotsec(
    sec: &State,
    nmodels: usize,
    scale: f64,
    ribb: bool,
    tips: bool,
    cg: bool,
    show_box: bool,
    color_rgb: [u8; 3],
) -> PlotScene3d {
    let mut scene = PlotScene3d::new();
    let wingspan = 1.85 * scale;

    if show_box {
        let (bmesh, blines) = boxtrace();
        scene.add_mesh(bmesh);
        for l in blines {
            scene.add_line(l);
        }
    }
    if cg {
        scene.add_line(cgtrace(sec, "CG", color_rgb, 2.5));
    }
    if tips {
        for t in tiptrace(sec, wingspan) {
            scene.add_line(t);
        }
    }
    if ribb && sec.len() >= 2 {
        scene.add_mesh(ribbon(sec, 0.5 * wingspan, color_rgb, 0.55, "Ribbon"));
    }
    if nmodels > 0 {
        for m in meshes(nmodels, sec, [249, 115, 22], scale, None) {
            scene.add_mesh(m);
        }
    }
    scene
}

/// Plot regions (e.g. `"manoeuvre"` or `"element"`) with distinct qualitative colors (`plots.plot_regions`).
pub fn plot_regions(st: &State, label_group_name: &str, span: f64) -> PlotScene3d {
    let mut scene = PlotScene3d::new();
    let Some(lg) = st.labels.get(label_group_name) else {
        return plotsec(st, 10, 1.0, true, true, true, true, get_colour_rgb(0));
    };
    for (i, (name, lab)) in lg.labels.iter().enumerate() {
        let seg = st.slice_time(lab.start, lab.stop);
        if seg.len() < 3 {
            continue;
        }
        let col = get_colour_rgb(i);
        scene.add_mesh(ribbon(&seg, span, col, 0.65, name.clone()));
        scene.add_line(Line3dTrace {
            name: name.clone(),
            points: seg.pos.data.clone(),
            color_rgb: col,
            width: 2.5,
            show_legend: true,
        });
    }
    scene
}

/// Create orthographic 3-view (`front`, `top`, `left`) states matching `plots.create_ortho_state`.
pub fn create_ortho_state(st: &State, gap: f64) -> [State; 3] {
    let xs = st.pos.x();
    let ys = st.pos.y();
    let zs = st.pos.z();
    let min_x = xs.iter().copied().fold(f64::INFINITY, f64::min);
    let max_x = xs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let min_y = ys.iter().copied().fold(f64::INFINITY, f64::min);
    let max_y = ys.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let min_z = zs.iter().copied().fold(f64::INFINITY, f64::min);
    let max_z = zs.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    let center = Point::new(
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    );
    let width_x = max_x - min_x;
    let width_y = max_y - min_y;
    let width_z = max_z - min_z;

    let front = st.move_state(&Transformation::new(-&center, euler_angles(0.0, 0.0, 0.0)));
    let top = front
        .move_state(&Transformation::new(p0(1), euler_angles(PI / 2.0, 0.0, 0.0)))
        .move_state(&Transformation::new(
            Point::new(0.0, 0.0, 0.5 * (width_y + width_z) + gap),
            euler_angles(0.0, 0.0, 0.0),
        ));
    let left = front
        .move_state(&Transformation::new(p0(1), euler_angles(0.0, 0.0, PI / 2.0)))
        .move_state(&Transformation::new(
            Point::new(-0.5 * (width_x + width_y) - gap, 0.0, 0.0),
            euler_angles(0.0, 0.0, 0.0),
        ));
    let _ = py;
    [front, top, left]
}

#[cfg(test)]
mod tests {
    use super::*;
    use pfc_flightdata::Flight;
    use std::path::PathBuf;

    fn test_state() -> State {
        let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../packages/Plotting/tests/data/p23_flight.json");
        let flight = Flight::from_json_file(&p).expect("load p23_flight.json");
        State::from_flight(&flight, None).expect("State::from_flight")
    }

    #[test]
    fn test_obj_loading() {
        let plane = OBJ::default_f3a_plane();
        assert!(!plane.vertices.is_empty());
        assert!(!plane.faces.is_empty());
    }

    #[test]
    fn test_plotsec_cg_and_cgtrace() {
        let st = test_state();
        let trace = cgtrace(&st, "CG", [99, 110, 250], 2.0);
        assert_eq!(trace.points.len(), st.len());

        let fig = plotsec(&st, 10, 1.0, true, false, true, true, [99, 110, 250]);
        assert!(!fig.lines.is_empty());
        assert!(!fig.meshes.is_empty());

        let html = fig.to_plotly_html("P23 Test Flight");
        assert!(html.contains("Plotly.newPlot"));
    }
}
