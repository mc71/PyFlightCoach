use crate::app::FlightCoachGui;
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int};
use std::slice;

pub struct FlightCoachHandle {
    pub gui: FlightCoachGui,
}

/// Create a new FlightCoach engine handle initialized with the embedded sample F3A P25 flight.
#[no_mangle]
pub extern "C" fn flightcoach_create() -> *mut FlightCoachHandle {
    let handle = Box::new(FlightCoachHandle {
        gui: FlightCoachGui::default(),
    });
    Box::into_raw(handle)
}

/// Destroy a FlightCoach engine handle created by `flightcoach_create`.
///
/// # Safety
/// `handle` must be a valid non-null pointer returned by `flightcoach_create`.
#[no_mangle]
pub unsafe extern "C" fn flightcoach_destroy(handle: *mut FlightCoachHandle) {
    if !handle.is_null() {
        drop(Box::from_raw(handle));
    }
}

/// Load a FlightCoach JSON (`FCJ`) string into the engine. Returns `0` on success, `-1` on error.
///
/// # Safety
/// `handle` and `json_utf8` must be valid pointers.
#[no_mangle]
pub unsafe extern "C" fn flightcoach_load_fcj_json(
    handle: *mut FlightCoachHandle,
    json_utf8: *const c_char,
) -> c_int {
    if handle.is_null() || json_utf8.is_null() {
        return -1;
    }
    let Ok(s) = CStr::from_ptr(json_utf8).to_str() else {
        return -1;
    };
    match (*handle).gui.load_fcj_str(s, "Mobile FCJ") {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

/// Load an ArduPilot DataFlash `.BIN` byte buffer into the engine. Returns `0` on success, `-1` on error.
///
/// # Safety
/// `handle` and `bytes` must be valid pointers for `len` bytes.
#[no_mangle]
pub unsafe extern "C" fn flightcoach_load_bin_bytes(
    handle: *mut FlightCoachHandle,
    bytes: *const u8,
    len: usize,
) -> c_int {
    if handle.is_null() || bytes.is_null() || len == 0 {
        return -1;
    }
    let slice = slice::from_raw_parts(bytes, len);
    match (*handle).gui.load_bin_bytes(slice, "Mobile BIN") {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

/// Return the number of samples in the active flight/manoeuvre state.
///
/// # Safety
/// `handle` must be a valid pointer.
#[no_mangle]
pub unsafe extern "C" fn flightcoach_active_sample_count(
    handle: *const FlightCoachHandle,
) -> usize {
    if handle.is_null() {
        return 0;
    }
    (*handle).gui.active_state.len()
}

/// Select a manoeuvre by index (`-1` selects the full flight).
///
/// # Safety
/// `handle` must be a valid pointer.
#[no_mangle]
pub unsafe extern "C" fn flightcoach_select_manoeuvre(
    handle: *mut FlightCoachHandle,
    manoeuvre_idx: c_int,
) {
    if handle.is_null() {
        return;
    }
    if manoeuvre_idx < 0 {
        (*handle).gui.select_manoeuvre(None);
    } else {
        (*handle).gui.select_manoeuvre(Some(manoeuvre_idx as usize));
    }
}

/// Run the native Rust `pfc-fcscore` grading engine on the currently loaded flight.
///
/// # Safety
/// `handle` must be a valid pointer.
#[no_mangle]
pub unsafe extern "C" fn flightcoach_run_fcscore(handle: *mut FlightCoachHandle) {
    if !handle.is_null() {
        (*handle).gui.run_fcscore_analysis();
    }
}

/// Export a standalone interactive 3D Plotly HTML document for the currently active scene.
/// Caller must free the returned string with `flightcoach_free_string`.
///
/// # Safety
/// `handle` must be a valid pointer.
#[no_mangle]
pub unsafe extern "C" fn flightcoach_export_plotly_html(
    handle: *const FlightCoachHandle,
) -> *mut c_char {
    if handle.is_null() {
        return std::ptr::null_mut();
    }
    let scene = (*handle).gui.build_current_scene();
    let html = scene.to_plotly_html(&(*handle).gui.loaded_title);
    CString::new(html)
        .map(|cs| cs.into_raw())
        .unwrap_or(std::ptr::null_mut())
}

/// Export a JSON summary of the loaded flight, manoeuvres, and F3A scores.
/// Caller must free the returned string with `flightcoach_free_string`.
///
/// # Safety
/// `handle` must be a valid pointer.
#[no_mangle]
pub unsafe extern "C" fn flightcoach_get_summary_json(
    handle: *const FlightCoachHandle,
) -> *mut c_char {
    if handle.is_null() {
        return std::ptr::null_mut();
    }
    let gui = &(*handle).gui;
    let scores: Vec<serde_json::Value> = gui
        .score_rows
        .iter()
        .map(|r| {
            serde_json::json!({
                "index": r.index,
                "name": r.name,
                "k": r.k,
                "total": r.total,
                "weighted": r.weighted,
            })
        })
        .collect();
    let summary = serde_json::json!({
        "title": gui.loaded_title,
        "samples": gui.active_state.len(),
        "duration_s": gui.active_state.duration(),
        "manoeuvres": gui.manoeuvres,
        "fa_version": gui.fa_version,
        "scores": scores,
    });
    CString::new(summary.to_string())
        .map(|cs| cs.into_raw())
        .unwrap_or(std::ptr::null_mut())
}

/// Free a C string returned by `flightcoach_export_plotly_html` or `flightcoach_get_summary_json`.
///
/// # Safety
/// `ptr` must have been allocated by `CString::into_raw`.
#[no_mangle]
pub unsafe extern "C" fn flightcoach_free_string(ptr: *mut c_char) {
    if !ptr.is_null() {
        drop(CString::from_raw(ptr));
    }
}

// ============================================================================
// Zero-dependency Android JNI entry points for `org.flightcoach.app.MainActivity`
// ============================================================================

#[repr(C)]
pub struct JniNativeInterface {
    _reserved: [*const std::ffi::c_void; 167],
    pub new_string_utf: unsafe extern "C" fn(*mut *const JniNativeInterface, *const c_char) -> *mut std::ffi::c_void,
}

pub type JNIEnv = *mut *const JniNativeInterface;
pub type JObject = *mut std::ffi::c_void;
pub type JString = *mut std::ffi::c_void;

#[no_mangle]
pub extern "C" fn Java_org_flightcoach_app_MainActivity_nativeCreate(
    _env: JNIEnv,
    _thiz: JObject,
) -> i64 {
    flightcoach_create() as usize as i64
}

#[no_mangle]
pub unsafe extern "C" fn Java_org_flightcoach_app_MainActivity_nativeDestroy(
    _env: JNIEnv,
    _thiz: JObject,
    handle: i64,
) {
    flightcoach_destroy(handle as usize as *mut FlightCoachHandle);
}

#[no_mangle]
pub unsafe extern "C" fn Java_org_flightcoach_app_MainActivity_nativeSelectManoeuvre(
    _env: JNIEnv,
    _thiz: JObject,
    handle: i64,
    index: i32,
) {
    flightcoach_select_manoeuvre(handle as usize as *mut FlightCoachHandle, index as c_int);
}

#[no_mangle]
pub unsafe extern "C" fn Java_org_flightcoach_app_MainActivity_nativeRunFcscore(
    _env: JNIEnv,
    _thiz: JObject,
    handle: i64,
) {
    flightcoach_run_fcscore(handle as usize as *mut FlightCoachHandle);
}

#[no_mangle]
pub unsafe extern "C" fn Java_org_flightcoach_app_MainActivity_nativeExportPlotlyHtml(
    env: JNIEnv,
    _thiz: JObject,
    handle: i64,
) -> JString {
    let c_ptr = flightcoach_export_plotly_html(handle as usize as *const FlightCoachHandle);
    if c_ptr.is_null() || env.is_null() || (*env).is_null() {
        return std::ptr::null_mut();
    }
    let jstr = ((**env).new_string_utf)(env, c_ptr);
    flightcoach_free_string(c_ptr);
    jstr
}

#[no_mangle]
pub unsafe extern "C" fn Java_org_flightcoach_app_MainActivity_nativeGetSummaryJson(
    env: JNIEnv,
    _thiz: JObject,
    handle: i64,
) -> JString {
    let c_ptr = flightcoach_get_summary_json(handle as usize as *const FlightCoachHandle);
    if c_ptr.is_null() || env.is_null() || (*env).is_null() {
        return std::ptr::null_mut();
    }
    let jstr = ((**env).new_string_utf)(env, c_ptr);
    flightcoach_free_string(c_ptr);
    jstr
}
