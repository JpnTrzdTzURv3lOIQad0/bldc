#include "pm_control.h"

#include <math.h>
#include <string.h>

static bool finite_nonnegative(float value) {
	return isfinite(value) && value >= 0.0f;
}

static bool config_is_valid(const pm_control_config_t *config) {
	return config != NULL &&
			isfinite(config->max_velocity_counts_per_second) &&
			config->max_velocity_counts_per_second > 0.0f &&
			finite_nonnegative(config->position_gain_per_second) &&
			finite_nonnegative(
					config->velocity_kp_amps_per_count_per_second) &&
			finite_nonnegative(config->velocity_ki_amps_per_count) &&
			isfinite(config->current_limit_amps) &&
			config->current_limit_amps > 0.0f &&
			finite_nonnegative(config->velocity_filter_time_constant_seconds) &&
			isfinite(config->max_dt_seconds) &&
			config->max_dt_seconds > 0.0f &&
			config->following_error_limit_counts > 0;
}

static bool subtract_i64(int64_t left, int64_t right, int64_t *result) {
	if ((right > 0 && left < INT64_MIN + right) ||
			(right < 0 && left > INT64_MAX + right)) {
		return false;
	}
	*result = left - right;
	return true;
}

static uint64_t magnitude_i64(int64_t value) {
	return value < 0 ? (uint64_t)(-(value + 1)) + 1U : (uint64_t)value;
}

static float clamp_float(float value, float limit) {
	if (value > limit) {
		return limit;
	}
	if (value < -limit) {
		return -limit;
	}
	return value;
}

pm_control_result_t pm_control_init(pm_control_t *control,
		const pm_control_config_t *config) {
	if (control == NULL || !config_is_valid(config)) {
		return PM_CONTROL_RESULT_INVALID_ARGUMENT;
	}
	memset(control, 0, sizeof(*control));
	control->config = *config;
	return PM_CONTROL_RESULT_OK;
}

pm_control_result_t pm_control_reset(pm_control_t *control,
		float measured_velocity_counts_per_second) {
	if (control == NULL ||
			!isfinite(measured_velocity_counts_per_second)) {
		return PM_CONTROL_RESULT_INVALID_ARGUMENT;
	}
	control->velocity_integral_amps = 0.0f;
	control->filtered_measured_velocity_counts_per_second =
			measured_velocity_counts_per_second;
	control->initialized = true;
	return PM_CONTROL_RESULT_OK;
}

pm_control_result_t pm_control_step(pm_control_t *control,
		const pm_control_input_t *input, float dt_seconds,
		pm_control_output_t *output) {
	if (output == NULL) {
		return PM_CONTROL_RESULT_INVALID_ARGUMENT;
	}
	memset(output, 0, sizeof(*output));
	if (control == NULL || input == NULL) {
		return PM_CONTROL_RESULT_INVALID_ARGUMENT;
	}
	if (!isfinite(dt_seconds) || dt_seconds <= 0.0f ||
			dt_seconds > control->config.max_dt_seconds) {
		return PM_CONTROL_RESULT_INVALID_DT;
	}
	if (!input->enabled || !input->feedback_valid || !input->referenced ||
			input->faulted) {
		control->velocity_integral_amps = 0.0f;
		control->filtered_measured_velocity_counts_per_second = 0.0f;
		control->initialized = false;
		return PM_CONTROL_RESULT_NOT_READY;
	}
	if (!isfinite(input->commanded_velocity_counts_per_second) ||
			!isfinite(input->measured_velocity_counts_per_second)) {
		return PM_CONTROL_RESULT_NUMERIC_ERROR;
	}

	pm_control_t candidate = *control;
	int64_t position_error;
	if (!subtract_i64(input->commanded_position_counts,
			input->measured_position_counts, &position_error)) {
		return PM_CONTROL_RESULT_NUMERIC_ERROR;
	}
	if (!candidate.initialized) {
		candidate.velocity_integral_amps = 0.0f;
		candidate.filtered_measured_velocity_counts_per_second =
				input->measured_velocity_counts_per_second;
		candidate.initialized = true;
	} else {
		float filter_alpha = candidate.config.velocity_filter_time_constant_seconds ==
				0.0f ? 1.0f : dt_seconds /
				(candidate.config.velocity_filter_time_constant_seconds +
				dt_seconds);
		candidate.filtered_measured_velocity_counts_per_second +=
				filter_alpha * (input->measured_velocity_counts_per_second -
				candidate.filtered_measured_velocity_counts_per_second);
	}

	float position_velocity = 0.0f;
	bool position_velocity_saturated = false;
	if (candidate.config.position_gain_per_second > 0.0f &&
			position_error != 0) {
		float error_limit = candidate.config.max_velocity_counts_per_second /
				candidate.config.position_gain_per_second;
		if (isfinite(error_limit) &&
				(float)magnitude_i64(position_error) >= error_limit) {
			position_velocity_saturated = true;
			position_velocity = position_error < 0 ?
					-candidate.config.max_velocity_counts_per_second :
				candidate.config.max_velocity_counts_per_second;
		} else {
			position_velocity = (float)position_error *
					candidate.config.position_gain_per_second;
		}
	}
	float requested_velocity = input->commanded_velocity_counts_per_second +
			position_velocity;
	if (!isfinite(position_velocity) || !isfinite(requested_velocity) ||
			!isfinite(candidate.filtered_measured_velocity_counts_per_second)) {
		return PM_CONTROL_RESULT_NUMERIC_ERROR;
	}
	float limited_velocity = clamp_float(requested_velocity,
			candidate.config.max_velocity_counts_per_second);
	float velocity_error = limited_velocity -
			candidate.filtered_measured_velocity_counts_per_second;
	float proportional_current = velocity_error *
			candidate.config.velocity_kp_amps_per_count_per_second;
	float unsaturated_current = proportional_current +
			candidate.velocity_integral_amps;
	if (!isfinite(limited_velocity) || !isfinite(velocity_error) ||
			!isfinite(proportional_current) || !isfinite(unsaturated_current)) {
		return PM_CONTROL_RESULT_NUMERIC_ERROR;
	}
	float limited_current = clamp_float(unsaturated_current,
			candidate.config.current_limit_amps);
	bool current_saturated = limited_current != unsaturated_current;
	float integral_delta = velocity_error *
			candidate.config.velocity_ki_amps_per_count * dt_seconds;
	float next_integral = candidate.velocity_integral_amps + integral_delta;
	if (!isfinite(integral_delta) || !isfinite(next_integral)) {
		return PM_CONTROL_RESULT_NUMERIC_ERROR;
	}
	bool integral_unwinds_saturation =
			(unsaturated_current > candidate.config.current_limit_amps &&
			velocity_error < 0.0f) ||
			(unsaturated_current < -candidate.config.current_limit_amps &&
			velocity_error > 0.0f);
	if (!current_saturated || integral_unwinds_saturation) {
		candidate.velocity_integral_amps = clamp_float(next_integral,
				candidate.config.current_limit_amps);
	}

	output->position_error_counts = position_error;
	output->commanded_velocity_counts_per_second = limited_velocity;
	output->filtered_measured_velocity_counts_per_second =
			candidate.filtered_measured_velocity_counts_per_second;
	output->velocity_error_counts_per_second = velocity_error;
	output->iq_demand_amps = limited_current;
	output->velocity_saturated = position_velocity_saturated ||
			limited_velocity != requested_velocity;
	output->current_saturated = current_saturated;
	output->following_error_exceeded = magnitude_i64(position_error) >
			(uint64_t)candidate.config.following_error_limit_counts;
	output->output_valid = true;
	*control = candidate;
	return PM_CONTROL_RESULT_OK;
}