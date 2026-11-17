# FlightCoach (Rust) — 3D Aerobatic Flight Plotter, Log Analyzer & FCScore Grader

A complete Rust rewrite of [PyFlightCoach](https://github.com/PyFlightCoach/PyFlightCoach), providing a unified Cargo workspace, native hardware-accelerated 3D aerobatic flight visualizer (`egui`/`eframe`), offline **FCScore** automatic flight grading engine (`pfc-fcscore`), standalone 3D Plotly HTML exporter, and C-FFI / JNI libraries compiled for **Desktop (Windows, macOS, Linux)**, **Android**, and **iOS**.

<p align="center">
  <img src="comet.png" alt="FlightCoach 3D Plot" width="49%"/>
  <img src="sql.png" alt="FlightCoach Analysis" width="49%"/>
</p>

---

## Key Features

- **Interactive 3D Aerobatic Box Viewport**:
  - Real-time 3D hardware-batched rendering of the `ColdDraftF3APlane.obj` aircraft model, 3D wingtip ribbon mesh, left/right wingtip trails, CG trajectory, and the $\pm 60^\circ$ F3A aerobatic box wedge.
  - Perspective and Orthographic projections with one-click camera presets (**Judge View**, **Top View**, **Side View**), mouse orbit/pan/zoom, adjustable aircraft scale, and sequence ghost models.
  - Interactive timeline playback scrubber (`0.5x`, `1x`, `2x`, `4x`) with live HUD readout of box position $(X, Y, Z)$, Euler attitude (Roll, Pitch, Yaw), ground speed, and normal load factor ($G$).
- **Native Rust FCScore Grading Engine (`pfc-fcscore`)**:
  - **Automatic Manoeuvre & Element Segmentation**: Automatically segments raw ArduPilot `.BIN` logs into schedule manoeuvres (`tkOf`, `1..17`, `lnd`) and splits each manoeuvre into principal elements (`ENTRY_LINE`, `LINE`, `ROLL`, `LOOP`, `SNAP`, `SPIN`, `STALLTURN`, `EXIT_LINE`) with local split-point optimization.
  - **6-Step Intra-Element Assessment**: Computes geometric measurements (`roll_angle`, `rf_y_track`, `rf_z_track`, `roll_rate`, `radius_curve_fit`, `loop_axial_track`, `stallturn_width`, `autorotation_rate`), weights errors by judge line-of-sight visibility, applies zero-phase Butterworth low-pass smoothing, and evaluates F3A downgrade criteria.
  - **Inter-Element & Box Positioning Assessment**: Evaluates loop radius matching, interior line length balance, multi-roll rate consistency, manoeuvre speed envelope, centreline symmetry ($X = 0$), $\pm 60^\circ$ lateral/vertical box boundary excursions, and flightline depth ($Y$) drift.
  - **Live Difficulty & Truncation Controls**: Supports instant switching between **Difficulty `1` (Easy)**, **`2` (Medium)**, and **`3` (Hard)** exponential curves, as well as **`0.5`-point truncation** (human judge rounding) and an interactive per-element downgrade breakdown inspector.
- **Synchronized Telemetry & Aerodynamic Charts**:
  - Time-synchronized plots of Euler attitude angles (Roll, Pitch, Yaw), box altitude $Z$ and ground speed, normal load factor ($G$), angle of attack ($\alpha$), and sideslip angle ($\beta$).
- **Multi-Format Flight Log Support**:
  - Pure-Rust ArduPilot DataFlash `.BIN` parser (no `pymavlink` or Python runtime required).
  - FlightCoach `.fc` / `.json` (`FCJ`) files, `.ajson` analysis logs (`AJson`), F3A Zone Pro `.f3a` box files, and flight CSVs.
- **Standalone 3D Plotly HTML Export**:
  - Export any full flight or selected manoeuvre to a self-contained interactive 3D Plotly HTML file from the GUI or headless CLI (`--export-html`).

---

## Workspace Architecture

```text
flightcoach/
├── Cargo.toml                        # Root Cargo workspace manifest
├── .github/workflows/ci.yml          # Multi-platform CI (Desktop, Android, iOS)
├── crates/
│   ├── pfc-schemas/                  # FlightCoach JSON/AJSON/SDef/Aresti Serde schemas
│   ├── pfc-geometry/                 # 3D math, Quaternions, WGS84/NED GPS, Mass, Air/Flow
│   ├── ardupilot-log-reader/         # Pure-Rust ArduPilot DataFlash .BIN binary log parser
│   ├── pfc-flightdata/               # Flight, Origin, Environment, State & sensor alignment
│   ├── pfc-fcscore/                  # Native Rust FCScore flight & element grading engine
│   ├── pfc-plotting/                 # 3D meshes, ribbons, wingtip trails & Plotly HTML export
│   ├── flightcoach-app/              # Desktop GUI (egui/eframe), CLI & C-FFI / Android JNI
│   ├── windows-link-shim/            # Self-contained Windows GNU/MSVC import library shim
│   └── webbrowser-shim/              # Cross-platform URL launcher shim
└── mobile/
    ├── android/                      # Android Studio Gradle project & Kotlin JNI MainActivity
    └── ios/                          # C FFI header (flightcoach.h) & SwiftUI wrapper
```

### Crate Summary

| Crate | Path | Description |
| :--- | :--- | :--- |
| **`pfc-schemas`** | [`crates/pfc-schemas`](crates/pfc-schemas) | Strongly-typed Serde data models for `.fc` / `.json` (`FCJ`), `.ajson` (`AJson`), `.maninfo.json` (`ManInfo`), `.sinfo.json` (`SInfo`), `.sdef.json` (`SDef`), Aresti figures (`Olan`, `Figure`, `Sequence`), and `MA` manoeuvre results. |
| **`pfc-geometry`** | [`crates/pfc-geometry`](crates/pfc-geometry) | Vectorized 3D geometry primitives (`Point`, `Quaternion`, `Coord`, `Transformation`, `Time`), WGS84/NED GPS projections (`GPS`), inertial tensors (`Mass`), Butterworth zero-phase `filtfilt` smoothing, cubic interpolation, and aerodynamics (`Air`). |
| **`ardupilot-log-reader`** | [`crates/ardupilot-log-reader`](crates/ardupilot-log-reader) | Pure-Rust ArduPilot DataFlash `.BIN` parser (`Ardupilot`). Dynamically parses `FMT` records, supports wildcard message selection (`XKF1*`, `GPS*`, `ATT`, `IMU`, `POS`, `ORGN`, `BARO`), computes GPS leap-second Unix timestamps, and extracts `PARM` parameters. |
| **`pfc-flightdata`** | [`crates/pfc-flightdata`](crates/pfc-flightdata) | Flight telemetry and kinematics (`Origin`, `Flight`, `Environment`, `Flow`, `State`, `LabelGroup`). Performs NED/Wind/Judges-Box coordinate transformations, manoeuvre slicing, state kinematic extrapolation, and multi-sensor temporal alignment. |
| **`pfc-fcscore`** | [`crates/pfc-fcscore`](crates/pfc-fcscore) | Offline FCScore grading engine (`score_flight_state`, `score_manoeuvre`). Segments flights and manoeuvres into principal elements and computes Intra, Inter, and Positioning downgrades across F3A schedules (`P25`, `P23`). |
| **`pfc-plotting`** | [`crates/pfc-plotting`](crates/pfc-plotting) | 3D aircraft mesh (`OBJ` with embedded `ColdDraftF3APlane.obj`), 3D wingtip ribbon generator (`ribbon`), wingtip trails (`tiptrace`), F3A aerobatic box wedge (`boxtrace`), 3-view orthographic states (`create_ortho_state`), and Plotly 3D HTML export (`PlotScene3d::to_plotly_html`). |
| **`flightcoach-app`** | [`crates/flightcoach-app`](crates/flightcoach-app) | Cross-platform Desktop GUI (`FlightCoachGui`), headless CLI exporter, and C-FFI / Android JNI library (`libflightcoach_app`). |

---

## Building & Running

### Prerequisites

- [Rust](https://rustup.rs/) (`stable` toolchain, edition 2021)
- Clone with submodules if running the upstream test fixtures:
  ```bash
  git clone --recurse-submodules https://github.com/mc71/PyFlightCoach.git
  cd PyFlightCoach
  ```

---

### 1. Desktop (Windows, macOS, Linux)

#### Run Workspace Tests
Run all 17 unit and integration tests across the 7 workspace crates:

```bash
cargo test --workspace
```

#### Build & Launch the Desktop GUI
Build and launch the interactive 3D Desktop application (pre-loaded with a sample F3A P25 competition flight and embedded ArduPilot `.BIN` log):

```bash
cargo run --release -p flightcoach-app
```

Or run the compiled release binary directly:
- **Windows**: `.\target\release\flightcoach.exe`
- **macOS / Linux**: `./target/release/flightcoach`

#### CLI Arguments
Open a specific `.BIN`, `.fc`, `.json`, `.ajson`, or `.f3a` file on startup:

```bash
cargo run --release -p flightcoach-app -- path/to/flight.BIN
```

Export a standalone interactive 3D Plotly HTML visualization headlessly from the command line:

```bash
cargo run --release -p flightcoach-app -- --export-html flight_plot.html [path/to/flight.json]
```

---

### 2. Android (`aarch64-linux-android` & `x86_64-linux-android`)

Add the Android Rust targets:

```bash
rustup target add aarch64-linux-android x86_64-linux-android
```

Compile the release static/Rust libraries directly:

```bash
cargo build --release --target aarch64-linux-android -p flightcoach-app --lib
cargo build --release --target x86_64-linux-android -p flightcoach-app --lib
```

To build the JNI shared libraries (`libflightcoach_app.so`) into [`mobile/android/app/src/main/jniLibs`](mobile/android/app/src/main/jniLibs) using [`cargo-ndk`](https://github.com/bbqsrc/cargo-ndk):

```bash
cargo install cargo-ndk
cargo ndk -t arm64-v8a -t x86_64 -o mobile/android/app/src/main/jniLibs rustc --release -p flightcoach-app --lib -- --crate-type=cdylib
```

Then open [`mobile/android`](mobile/android) in Android Studio or build via Gradle (`./gradlew assembleDebug`). [`MainActivity.kt`](mobile/android/app/src/main/java/org/flightcoach/app/MainActivity.kt) links directly against `libflightcoach_app.so` via zero-dependency JNI bindings (`nativeCreate`, `nativeSelectManoeuvre`, `nativeRunFcscore`, `nativeExportPlotlyHtml`, `nativeGetSummaryJson`).

---

### 3. iOS (`aarch64-apple-ios` & `aarch64-apple-ios-sim`)

Add the iOS Rust targets:

```bash
rustup target add aarch64-apple-ios aarch64-apple-ios-sim
```

Compile the release static libraries (`libflightcoach_app.a`) for iOS devices (ARM64) and Apple Silicon iOS Simulator:

```bash
cargo build --release --target aarch64-apple-ios -p flightcoach-app --lib
cargo build --release --target aarch64-apple-ios-sim -p flightcoach-app --lib
```

On macOS with Xcode installed, bundle the static libraries and C header [`mobile/ios/flightcoach.h`](mobile/ios/flightcoach.h) into an `.xcframework`:

```bash
xcodebuild -create-xcframework \
  -library target/aarch64-apple-ios/release/libflightcoach_app.a \
  -headers mobile/ios \
  -library target/aarch64-apple-ios-sim/release/libflightcoach_app.a \
  -headers mobile/ios \
  -output FlightCoach.xcframework
```

Import `FlightCoach.xcframework` and [`mobile/ios/FlightCoachApp.swift`](mobile/ios/FlightCoachApp.swift) into Xcode to build and deploy to iPhone or iPad.

---

## C-FFI API Reference (`flightcoach.h`)

The `flightcoach-app` library exposes a clean C ABI in [`crates/flightcoach-app/src/ffi.rs`](crates/flightcoach-app/src/ffi.rs) and [`mobile/ios/flightcoach.h`](mobile/ios/flightcoach.h):

```c
typedef struct FlightCoachHandle FlightCoachHandle;

// Lifecycle
FlightCoachHandle* flightcoach_create(void);
void flightcoach_destroy(FlightCoachHandle* handle);

// Data Loading
int flightcoach_load_fcj_json(FlightCoachHandle* handle, const char* json_utf8);
int flightcoach_load_bin_bytes(FlightCoachHandle* handle, const uint8_t* bytes, size_t len);

// Manoeuvre Selection & Offline FCScore Grading
size_t flightcoach_active_sample_count(const FlightCoachHandle* handle);
void flightcoach_select_manoeuvre(FlightCoachHandle* handle, int manoeuvre_idx);
void flightcoach_run_fcscore(FlightCoachHandle* handle);

// 3D Plotly HTML & JSON Score Summary Export
char* flightcoach_export_plotly_html(const FlightCoachHandle* handle);
char* flightcoach_get_summary_json(const FlightCoachHandle* handle);
void flightcoach_free_string(char* ptr);
```

---

## License

Licensed under the **GNU General Public License v3.0 or later (`GPL-3.0-or-later`)**, matching the upstream PyFlightCoach packages.
