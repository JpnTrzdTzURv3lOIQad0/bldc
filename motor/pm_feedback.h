#ifndef PM_FEEDBACK_H
#define PM_FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	PM_FEEDBACK_RESULT_OK = 0,
	PM_FEEDBACK_RESULT_NOT_READY,
	PM_FEEDBACK_RESULT_INVALID_ARGUMENT,
	PM_FEEDBACK_RESULT_IDENTITY_MISMATCH,
	PM_FEEDBACK_RESULT_STALE,
	PM_FEEDBACK_RESULT_SENSOR_FAULT,
	PM_FEEDBACK_RESULT_COMMUTATION_INVALID,
	PM_FEEDBACK_RESULT_AMBIGUOUS,
	PM_FEEDBACK_RESULT_IMPOSSIBLE_MOTION,
	PM_FEEDBACK_RESULT_RESET,
	PM_FEEDBACK_RESULT_OVERFLOW
} pm_feedback_result_t;

typedef struct {
	uint16_t axis_id;
	uint16_t source_id;
	uint32_t counts_per_revolution;
	uint32_t max_counts_per_second;
	uint32_t max_sample_gap_ms;
	uint32_t max_sample_age_ms;
	bool inverted;
} pm_feedback_config_t;

typedef struct {
	uint16_t axis_id;
	uint16_t source_id;
	uint32_t sequence;
	uint32_t acquisition_time_ms;
	uint32_t reset_epoch;
	uint32_t position_counts;
	bool sensor_healthy;
	bool commutation_valid;
} pm_feedback_sample_t;

typedef struct {
	uint16_t axis_id;
	uint16_t source_id;
	uint32_t sequence;
	uint32_t acquisition_time_ms;
	uint32_t reset_epoch;
	uint32_t sample_age_ms;
	int64_t raw_position_counts;
	int64_t reference_offset_counts;
	int64_t axis_position_counts;
	float velocity_counts_per_second;
	bool initialized;
	bool valid;
	bool commutation_valid;
	bool referenced;
} pm_feedback_status_t;

typedef struct {
	pm_feedback_config_t config;
	uint32_t last_raw_counts;
	uint32_t last_sequence;
	uint32_t last_acquisition_time_ms;
	uint32_t reset_epoch;
	int64_t raw_position_counts;
	int64_t reference_offset_counts;
	float velocity_counts_per_second;
	bool initialized;
	bool valid;
	bool commutation_valid;
	bool referenced;
} pm_feedback_t;

pm_feedback_result_t pm_feedback_init(pm_feedback_t *feedback,
		const pm_feedback_config_t *config);
pm_feedback_result_t pm_feedback_update(pm_feedback_t *feedback,
		const pm_feedback_sample_t *sample, uint32_t now_ms);
pm_feedback_result_t pm_feedback_check_freshness(pm_feedback_t *feedback,
		uint32_t now_ms);
pm_feedback_result_t pm_feedback_set_reference(pm_feedback_t *feedback,
		int64_t axis_position_counts, uint32_t now_ms);
pm_feedback_result_t pm_feedback_get_status(pm_feedback_t *feedback,
		uint32_t now_ms, pm_feedback_status_t *status);

#endif