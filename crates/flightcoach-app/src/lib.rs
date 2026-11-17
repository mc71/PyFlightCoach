pub mod app;
pub mod ffi;

pub use app::{ActiveTab, FlightCoachGui, ScoreRow, SAMPLE_BIN_LOG, SAMPLE_FCJ_JSON};
pub use ffi::*;

#[cfg(test)]
mod tests {
    use super::*;
    use std::ffi::CStr;

    #[test]
    fn test_gui_default_and_ffi() {
        unsafe {
            let handle = flightcoach_create();
            assert!(!handle.is_null());
            let count = flightcoach_active_sample_count(handle);
            assert!(count > 50);

            flightcoach_select_manoeuvre(handle, -1);
            let full_count = flightcoach_active_sample_count(handle);
            assert!(full_count > count);

            let summary_ptr = flightcoach_get_summary_json(handle);
            assert!(!summary_ptr.is_null());
            let summary_str = CStr::from_ptr(summary_ptr).to_str().unwrap();
            assert!(summary_str.contains("trgle"));
            flightcoach_free_string(summary_ptr);

            let html_ptr = flightcoach_export_plotly_html(handle);
            assert!(!html_ptr.is_null());
            let html_str = CStr::from_ptr(html_ptr).to_str().unwrap();
            assert!(html_str.contains("Plotly.newPlot"));
            flightcoach_free_string(html_ptr);

            // Verify native Rust FCScore re-scoring and BIN log auto-scoring
            flightcoach_run_fcscore(handle);
            let bin_rc = flightcoach_load_bin_bytes(
                handle,
                SAMPLE_BIN_LOG.as_ptr(),
                SAMPLE_BIN_LOG.len(),
            );
            assert_eq!(bin_rc, 0);
            let bin_summary_ptr = flightcoach_get_summary_json(handle);
            let bin_summary_str = CStr::from_ptr(bin_summary_ptr).to_str().unwrap();
            assert!(bin_summary_str.contains("rust-fcscore-1.0.0"));
            flightcoach_free_string(bin_summary_ptr);

            flightcoach_destroy(handle);
        }
    }
}
