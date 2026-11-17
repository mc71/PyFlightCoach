#[cfg(not(any(target_os = "android", target_os = "ios")))]
fn main() -> eframe::Result<()> {
    let args: Vec<String> = std::env::args().collect();

    // Support headless CLI export if `--export-html <output.html> [input_file]` is passed
    if let Some(pos) = args.iter().position(|a| a == "--export-html") {
        let out_path = args
            .get(pos + 1)
            .expect("Expected output path after --export-html");
        let mut gui = flightcoach_app::FlightCoachGui::default();
        if let Some(in_path) = args.get(pos + 2) {
            gui.load_file_path(std::path::Path::new(in_path))
                .expect("Failed to load input file");
        }
        let html = gui
            .build_current_scene()
            .to_plotly_html(&gui.loaded_title);
        std::fs::write(out_path, html).expect("Failed to write HTML output");
        println!("Exported 3D Plotly HTML to {}", out_path);
        return Ok(());
    }

    let mut gui = flightcoach_app::FlightCoachGui::default();
    if let Some(first_arg) = args.get(1) {
        let p = std::path::Path::new(first_arg);
        if p.exists() {
            let _ = gui.load_file_path(p);
        }
    }

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1360.0, 840.0])
            .with_title("FlightCoach — 3D Aerobatic Flight Plotter & Log Analyzer (Rust)"),
        ..Default::default()
    };

    eframe::run_native(
        "FlightCoach",
        options,
        Box::new(|_cc| Ok(Box::new(gui))),
    )
}

#[cfg(any(target_os = "android", target_os = "ios"))]
fn main() {}
