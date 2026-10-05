#include "pm_feedback.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#define PM_FEEDBACK_MAX_TIME_DELTA_US 0x7FFFFFFFU

static bool config_is_valid(const pm_feedback_config_t *config) {
	return config != NULL && config->counts_per_revolution >= 2 &&
			config->max_counts_per_second > 0 &&
			config->max_sample_gap_us > 0 &&
			config->max_sample_gap_us <= PM_FEEDBACK_MAX_TIME_DELTA_US &&
			config->max_sample_age_us > 0 &&
			config->max_sample_age_us <= PM_FEEDBACK_MAX_TIME_DELTA_US &&
			(uint64_t)config->max_counts_per_second * config->max_sample_gap_us * 2U <
			(uint64_t)config->counts_per_revolution * 1000000U;
}

static bool timestamp_is_fresh(uint32_t now_us, uint32_t timestamp_us,
		uint32_t max_age_us) {
	uint32_t age_us = now_us - timestamp_us;
	return age_us <= max_age_us && age_us <= PM_FEEDBACK_MAX_TIME_DELTA_US;
}

static bool add_i64(int64_t left, int64_t right, int64_t *result) {
	if ((right > 0 && left > INT64_MAX - right) ||
			(right < 0 && left < INT64_MIN - right)) {
		return false;
	}
	*result = left + right;
	return true;
}

static bool subtract_i64(int64_t left, int64_t right, int64_t *result) {
	if ((right > 0 && left < INT64_MIN + right) ||
			(right < 0 && left > INT64_MAX + right)) {
		return false;
	}
	*result = left - right;
	return true;
}

static void invalidate_tracking(pm_feedback_t *feedback) {
	feedback->initialized = false;
	feedback->valid = false;
	feedback->commutation_valid = false;
	feedback->referenced = false;
	feedback->velocity_counts_per_second = 0.0f;
}

static void accept_baseline(pm_feedback_t *feedback,
		const pm_feedback_sample_t *sample) {
	int64_t raw_position = (int64_t)sample->position_counts;
	if (feedback->config.inverted) {
		raw_position = -raw_position;
	}
	feedback->last_raw_counts = sample->position_counts;
	feedback->last_sequence = sample->sequence;
	feedback->last_acquisition_time_us = sample->acquisition_time_us;
	feedback->reset_epoch = sample->reset_epoch;
	feedback->raw_position_counts = raw_position;
	feedback->velocity_counts_per_second = 0.0f;
	feedback->initialized = true;
	feedback->valid = true;
	feedback->commutation_valid = sample->commutation_valid;
	feedback->referenced = false;
}

pm_feedback_result_t pm_feedback_init(pm_feedback_t *feedback,
		const pm_feedback_config_t *config) {
	if (feedback == NULL || !config_is_valid(config)) {
		return PM_FEEDBACK_RESULT_INVALID_ARGUMENT;
	}
	memset(feedback, 0, sizeof(*feedback));
	feedback->config = *config;
	return PM_FEEDBACK_RESULT_OK;
}

pm_feedback_result_t pm_feedback_update(pm_feedback_t *feedback,
		const pm_feedback_sample_t *sample, uint32_t now_us) {
	if (feedback == NULL || sample == NULL) {
		return PM_FEEDBACK_RESULT_INVALID_ARGUMENT;
	}
	if (sample->axis_id != feedback->config.axis_id ||
			sample->source_id != feedback->config.source_id) {
		return PM_FEEDBACK_RESULT_IDENTITY_MISMATCH;
	}
	if (!sample->sensor_healthy || sample->position_counts >=
			feedback->config.counts_per_revolution) {
		invalidate_tracking(feedback);
		return PM_FEEDBACK_RESULT_SENSOR_FAULT;
	}
	if (!timestamp_is_fresh(now_us, sample->acquisition_time_us,
			feedback->config.max_sample_age_us)) {
		invalidate_tracking(feedback);
		return PM_FEEDBACK_RESULT_STALE;
	}
	if (!feedback->initialized || !feedback->valid) {
		accept_baseline(feedback, sample);
		return PM_FEEDBACK_RESULT_OK;
	}
	if (sample->reset_epoch != feedback->reset_epoch) {
		accept_baseline(feedback, sample);
		return PM_FEEDBACK_RESULT_RESET;
	}

	uint32_t sequence_delta = sample->sequence - feedback->last_sequence;
	if (sequence_delta == 0 || sequence_delta > PM_FEEDBACK_MAX_TIME_DELTA_US) {
		return PM_FEEDBACK_RESULT_STALE;
	}
	uint32_t elapsed_us = sample->acquisition_time_us -
			feedback->last_acquisition_time_us;
	if (elapsed_us == 0 || elapsed_us > feedback->config.max_sample_gap_us ||
			elapsed_us > PM_FEEDBACK_MAX_TIME_DELTA_US) {
		invalidate_tracking(feedback);
		return PM_FEEDBACK_RESULT_STALE;
	}

	int64_t delta_counts = (int64_t)sample->position_counts -
			(int64_t)feedback->last_raw_counts;
	int64_t doubled_delta = delta_counts * 2;
	int64_t revolution = (int64_t)feedback->config.counts_per_revolution;
	if (doubled_delta == revolution || doubled_delta == -revolution) {
		invalidate_tracking(feedback);
		return PM_FEEDBACK_RESULT_AMBIGUOUS;
	}
	if (doubled_delta > revolution) {
		delta_counts -= revolution;
	} else if (doubled_delta < -revolution) {
		delta_counts += revolution;
	}
	if (feedback->config.inverted) {
		delta_counts = -delta_counts;
	}

	uint64_t magnitude = delta_counts < 0 ?
			(uint64_t)(-delta_counts) : (uint64_t)delta_counts;
	uint64_t allowed_counts =
			(uint64_t)feedback->config.max_counts_per_second * elapsed_us /
			1000000U + 1U;
	if (magnitude > allowed_counts) {
		invalidate_tracking(feedback);
		return PM_FEEDBACK_RESULT_IMPOSSIBLE_MOTION;
	}

	int64_t unwrapped_position;
	if (!add_i64(feedback->raw_position_counts, delta_counts,
			&unwrapped_position)) {
		invalidate_tracking(feedback);
		return PM_FEEDBACK_RESULT_OVERFLOW;
	}
	float velocity = (float)delta_counts * 1000000.0f / (float)elapsed_us;
	if (!isfinite(velocity)) {
		invalidate_tracking(feedback);
		return PM_FEEDBACK_RESULT_OVERFLOW;
	}
	feedback->raw_position_counts = unwrapped_position;
	feedback->velocity_counts_per_second = velocity;
	feedback->last_raw_counts = sample->position_counts;
	feedback->last_sequence = sample->sequence;
	feedback->last_acquisition_time_us = sample->acquisition_time_us;
	feedback->commutation_valid = sample->commutation_valid;
	if (!sample->commutation_valid) {
		feedback->referenced = false;
		return PM_FEEDBACK_RESULT_COMMUTATION_INVALID;
	}
	return PM_FEEDBACK_RESULT_OK;
}

pm_feedback_result_t pm_feedback_check_freshness(pm_feedback_t *feedback,
		uint32_t now_us) {
	if (feedback == NULL) {
		return PM_FEEDBACK_RESULT_INVALID_ARGUMENT;
	}
	if (!feedback->initialized || !feedback->valid) {
		return PM_FEEDBACK_RESULT_NOT_READY;
	}
	if (!timestamp_is_fresh(now_us, feedback->last_acquisition_time_us,
			feedback->config.max_sample_age_us)) {
		invalidate_tracking(feedback);
		return PM_FEEDBACK_RESULT_STALE;
	}
	return PM_FEEDBACK_RESULT_OK;
}

pm_feedback_result_t pm_feedback_set_reference(pm_feedback_t *feedback,
		int64_t axis_position_counts, uint32_t now_us) {
	if (feedback == NULL) {
		return PM_FEEDBACK_RESULT_INVALID_ARGUMENT;
	}
	pm_feedback_result_t freshness_result =
			pm_feedback_check_freshness(feedback, now_us);
	if (freshness_result != PM_FEEDBACK_RESULT_OK) {
		return freshness_result;
	}
	if (!feedback->commutation_valid) {
		return PM_FEEDBACK_RESULT_NOT_READY;
	}
	int64_t reference_offset;
	if (!subtract_i64(feedback->raw_position_counts, axis_position_counts,
			&reference_offset)) {
		return PM_FEEDBACK_RESULT_OVERFLOW;
	}
	feedback->reference_offset_counts = reference_offset;
	feedback->referenced = true;
	return PM_FEEDBACK_RESULT_OK;
}

pm_feedback_result_t pm_feedback_get_status(pm_feedback_t *feedback,
		uint32_t now_us, pm_feedback_status_t *status) {
	if (feedback == NULL || status == NULL) {
		return PM_FEEDBACK_RESULT_INVALID_ARGUMENT;
	}
	pm_feedback_result_t result = pm_feedback_check_freshness(feedback, now_us);
	memset(status, 0, sizeof(*status));
	status->axis_id = feedback->config.axis_id;
	status->source_id = feedback->config.source_id;
	status->sequence = feedback->last_sequence;
	status->acquisition_time_us = feedback->last_acquisition_time_us;
	status->reset_epoch = feedback->reset_epoch;
	status->sample_age_us = now_us - feedback->last_acquisition_time_us;
	status->raw_position_counts = feedback->raw_position_counts;
	status->reference_offset_counts = feedback->reference_offset_counts;
	status->velocity_counts_per_second =
			feedback->velocity_counts_per_second;
	status->initialized = feedback->initialized;
	status->valid = feedback->valid;
	status->commutation_valid = feedback->commutation_valid;
	status->referenced = feedback->referenced;
	if (feedback->referenced) {
		if (!subtract_i64(feedback->raw_position_counts,
				feedback->reference_offset_counts,
				&status->axis_position_counts)) {
			invalidate_tracking(feedback);
			status->valid = false;
			status->referenced = false;
			return PM_FEEDBACK_RESULT_OVERFLOW;
		}
	}
	return result;
}