#ifndef FLIGHTCOACH_H
#define FLIGHTCOACH_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct FlightCoachHandle FlightCoachHandle;

FlightCoachHandle* flightcoach_create(void);
void flightcoach_destroy(FlightCoachHandle* handle);

int flightcoach_load_fcj_json(FlightCoachHandle* handle, const char* json_utf8);
int flightcoach_load_bin_bytes(FlightCoachHandle* handle, const uint8_t* bytes, size_t len);

size_t flightcoach_active_sample_count(const FlightCoachHandle* handle);
void flightcoach_select_manoeuvre(FlightCoachHandle* handle, int manoeuvre_idx);
void flightcoach_run_fcscore(FlightCoachHandle* handle);

char* flightcoach_export_plotly_html(const FlightCoachHandle* handle);
char* flightcoach_get_summary_json(const FlightCoachHandle* handle);
void flightcoach_free_string(char* ptr);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHTCOACH_H */
