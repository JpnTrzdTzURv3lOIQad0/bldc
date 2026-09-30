#ifndef PM_CONTROL_H
#define PM_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	PM_CONTROL_RESULT_OK = 0,
	PM_CONTROL_RESULT_NOT_READY,
	PM_CONTROL_RESULT_INVALID_ARGUMENT,
	PM_CONTROL_RESULT_INVALID_DT,
	PM_CONTROL_RESULT_NUMERIC_ERROR
} pm_control_result_t;

typedef struct {
	float max_velocity_counts_per_second;
	float position_gain_per_second;
	float velocity_kp_amps_per_count_per_second;
	float velocity_ki_amps_per_count;
	float current_limit_amps;
	float velocity_filter_time_constant_seconds;
	float max_dt_seconds;
	int64_t following_error_limit_counts;
} pm_control_config_t;

typedef struct {
	int64_t commanded_position_counts;
	int64_t measured_position_counts;
	float commanded_velocity_counts_per_second;
	float measured_velocity_counts_per_second;
	bool enabled;
	bool feedback_valid;
	bool referenced;
	bool faulted;
} pm_control_input_t;

typedef struct {
	int64_t position_error_counts;
	float commanded_velocity_counts_per_second;
	float filtered_measured_velocity_counts_per_second;
	float velocity_error_counts_per_second;
	float iq_demand_amps;
	bool velocity_saturated;
	bool current_saturated;
	bool following_error_exceeded;
	bool output_valid;
} pm_control_output_t;

typedef struct {
	pm_control_config_t config;
	float velocity_integral_amps;
	float filtered_measured_velocity_counts_per_second;
	bool initialized;
} pm_control_t;

pm_control_result_t pm_control_init(pm_control_t *control,
		const pm_control_config_t *config);
pm_control_result_t pm_control_reset(pm_control_t *control,
		float measured_velocity_counts_per_second);
pm_control_result_t pm_control_step(pm_control_t *control,
		const pm_control_input_t *input, float dt_seconds,
		pm_control_output_t *output);

#endif