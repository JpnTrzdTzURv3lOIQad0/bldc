#include "pm_trajectory.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

static bool finite_positive(float value) {
	return isfinite(value) && value > 0.0f;
}

static bool append_phase(pm_trajectory_t *trajectory, float acceleration,
		float duration, float end_velocity) {
	if (!isfinite(acceleration) || !finite_positive(duration) ||
			!isfinite(end_velocity) ||
			trajectory->phase_count >= PM_TRAJECTORY_MAX_PHASES) {
		return false;
	}
	float end_position = trajectory->position +
			(trajectory->velocity + end_velocity) * 0.5f * duration;
	if (!isfinite(end_position)) {
		return false;
	}
	pm_trajectory_phase_t *phase =
			&trajectory->phases[trajectory->phase_count++];
	phase->duration = duration;
	phase->acceleration = acceleration;
	phase->end_position = end_position;
	phase->end_velocity = end_velocity;
	trajectory->position = end_position;
	trajectory->velocity = end_velocity;
	return true;
}

static bool append_rest_to_target(pm_trajectory_t *trajectory,
		float target_position, float direction) {
	float distance = direction * (target_position - trajectory->position);
	float initial_speed = direction * trajectory->velocity;
	if (!isfinite(distance) || !isfinite(initial_speed) || distance < 0.0f ||
			initial_speed < 0.0f) {
		return false;
	}
	if (distance == 0.0f && initial_speed == 0.0f) {
		return true;
	}

	float stop_distance = initial_speed * initial_speed /
			(2.0f * trajectory->max_deceleration);
	if (!isfinite(stop_distance) || distance < stop_distance) {
		return false;
	}
	float distance_at_max =
			(trajectory->max_velocity * trajectory->max_velocity -
			initial_speed * initial_speed) /
			(2.0f * trajectory->max_acceleration) +
			trajectory->max_velocity * trajectory->max_velocity /
			(2.0f * trajectory->max_deceleration);
	if (!isfinite(distance_at_max)) {
		return false;
	}

	float peak_speed;
	float cruise_distance = 0.0f;
	if (distance >= distance_at_max) {
		peak_speed = trajectory->max_velocity;
		float acceleration_distance =
				(peak_speed * peak_speed - initial_speed * initial_speed) /
				(2.0f * trajectory->max_acceleration);
		float deceleration_distance = peak_speed * peak_speed /
				(2.0f * trajectory->max_deceleration);
		cruise_distance = distance - acceleration_distance -
				deceleration_distance;
	} else {
		float peak_squared =
				(2.0f * distance * trajectory->max_acceleration *
				trajectory->max_deceleration +
				trajectory->max_deceleration * initial_speed * initial_speed) /
				(trajectory->max_acceleration +
				trajectory->max_deceleration);
		if (!isfinite(peak_squared) || peak_squared < 0.0f) {
			return false;
		}
		peak_speed = sqrtf(peak_squared);
	}
	if (!isfinite(peak_speed) || !isfinite(cruise_distance)) {
		return false;
	}
	float distance_tolerance = 8.0f * FLT_EPSILON *
			fmaxf(1.0f, fmaxf(distance, distance_at_max));
	if (cruise_distance < 0.0f &&
			cruise_distance >= -distance_tolerance) {
		cruise_distance = 0.0f;
	}
	float speed_tolerance = 8.0f * FLT_EPSILON *
			fmaxf(1.0f, initial_speed);
	if (peak_speed < initial_speed &&
			peak_speed >= initial_speed - speed_tolerance) {
		peak_speed = initial_speed;
	}
	if (peak_speed < initial_speed || cruise_distance < 0.0f) {
		return false;
	}

	if (peak_speed > initial_speed) {
		float duration = (peak_speed - initial_speed) /
				trajectory->max_acceleration;
		if (!append_phase(trajectory,
				direction * trajectory->max_acceleration, duration,
				direction * peak_speed)) {
			return false;
		}
	}
	if (cruise_distance > 0.0f) {
		float duration = cruise_distance / peak_speed;
		if (!append_phase(trajectory, 0.0f, duration,
				direction * peak_speed)) {
			return false;
		}
	}
	if (peak_speed > 0.0f) {
		float duration = peak_speed / trajectory->max_deceleration;
		if (!append_phase(trajectory,
				-direction * trajectory->max_deceleration, duration, 0.0f)) {
			return false;
		}
	}
	if (trajectory->phase_count > 0) {
		trajectory->phases[trajectory->phase_count - 1].end_position =
				target_position;
	}
	trajectory->position = target_position;
	trajectory->velocity = 0.0f;
	return true;
}

static pm_trajectory_result_t build_position_profile(
		pm_trajectory_t *trajectory, float current_position,
		float current_velocity, float target_position, float max_velocity,
		float max_acceleration, float max_deceleration, float max_dt) {
	if (!isfinite(current_position) || !isfinite(current_velocity) ||
			!isfinite(target_position) || !finite_positive(max_velocity) ||
			!finite_positive(max_acceleration) ||
			!finite_positive(max_deceleration) || !finite_positive(max_dt)) {
		return PM_TRAJECTORY_RESULT_INVALID_ARGUMENT;
	}
	memset(trajectory, 0, sizeof(*trajectory));
	trajectory->position = current_position;
	trajectory->velocity = current_velocity;
	trajectory->target_position = target_position;
	trajectory->max_velocity = max_velocity;
	trajectory->max_acceleration = max_acceleration;
	trajectory->max_deceleration = max_deceleration;
	trajectory->max_dt = max_dt;

	float delta = target_position - current_position;
	if (!isfinite(delta)) {
		return PM_TRAJECTORY_RESULT_NUMERIC_ERROR;
	}
	float direction = delta > 0.0f ? 1.0f :
			delta < 0.0f ? -1.0f : current_velocity > 0.0f ? -1.0f : 1.0f;
	float speed_toward_target = direction * current_velocity;
	if (!isfinite(speed_toward_target)) {
		return PM_TRAJECTORY_RESULT_NUMERIC_ERROR;
	}

	if (speed_toward_target < 0.0f) {
		float duration = -speed_toward_target / max_deceleration;
		if (!append_phase(trajectory, direction * max_deceleration, duration,
				0.0f)) {
			return PM_TRAJECTORY_RESULT_NUMERIC_ERROR;
		}
	} else if (speed_toward_target > max_velocity) {
		float duration = (speed_toward_target - max_velocity) /
				max_deceleration;
		if (!append_phase(trajectory, -direction * max_deceleration, duration,
				direction * max_velocity)) {
			return PM_TRAJECTORY_RESULT_NUMERIC_ERROR;
		}
	}

	float remaining = direction * (target_position - trajectory->position);
	float speed = direction * trajectory->velocity;
	float stopping_distance = speed * speed / (2.0f * max_deceleration);
	if (!isfinite(remaining) || !isfinite(speed) ||
			!isfinite(stopping_distance)) {
		return PM_TRAJECTORY_RESULT_NUMERIC_ERROR;
	}
	if (remaining <= stopping_distance && speed > 0.0f) {
		float duration = speed / max_deceleration;
		if (!append_phase(trajectory, -direction * max_deceleration,
				duration, 0.0f)) {
			return PM_TRAJECTORY_RESULT_NUMERIC_ERROR;
		}
		delta = target_position - trajectory->position;
		if (!isfinite(delta)) {
			return PM_TRAJECTORY_RESULT_NUMERIC_ERROR;
		}
		direction = delta > 0.0f ? 1.0f : delta < 0.0f ? -1.0f : 0.0f;
		if (direction != 0.0f && !append_rest_to_target(trajectory,
				target_position, direction)) {
			return PM_TRAJECTORY_RESULT_NUMERIC_ERROR;
		}
	} else if (remaining > 0.0f) {
		if (!append_rest_to_target(trajectory, target_position, direction)) {
			return PM_TRAJECTORY_RESULT_NUMERIC_ERROR;
		}
	}

	if (trajectory->phase_count > 0) {
		trajectory->phases[trajectory->phase_count - 1].end_position =
				target_position;
		trajectory->phases[trajectory->phase_count - 1].end_velocity = 0.0f;
	}
	trajectory->position = current_position;
	trajectory->velocity = current_velocity;
	trajectory->phase_elapsed = 0.0f;
	trajectory->phase_index = 0;
	trajectory->state = trajectory->phase_count == 0 ?
			PM_TRAJECTORY_COMPLETE : PM_TRAJECTORY_RUNNING;
	return trajectory->state == PM_TRAJECTORY_COMPLETE ?
			PM_TRAJECTORY_RESULT_COMPLETE : PM_TRAJECTORY_RESULT_RUNNING;
}

pm_trajectory_result_t pm_trajectory_start(pm_trajectory_t *trajectory,
		float current_position, float current_velocity, float target_position,
		float max_velocity, float max_acceleration, float max_deceleration,
		float max_dt) {
	if (trajectory == NULL) {
		return PM_TRAJECTORY_RESULT_INVALID_ARGUMENT;
	}
	pm_trajectory_t candidate;
	pm_trajectory_result_t result = build_position_profile(&candidate,
			current_position, current_velocity, target_position, max_velocity,
			max_acceleration, max_deceleration, max_dt);
	if (result == PM_TRAJECTORY_RESULT_RUNNING ||
			result == PM_TRAJECTORY_RESULT_COMPLETE) {
		*trajectory = candidate;
	}
	return result;
}

pm_trajectory_result_t pm_trajectory_retarget(pm_trajectory_t *trajectory,
		float target_position, float max_velocity, float max_acceleration,
		float max_deceleration, float max_dt) {
	if (trajectory == NULL || trajectory->state == PM_TRAJECTORY_INACTIVE) {
		return PM_TRAJECTORY_RESULT_INVALID_ARGUMENT;
	}
	return pm_trajectory_start(trajectory, trajectory->position,
			trajectory->velocity, target_position, max_velocity,
			max_acceleration, max_deceleration, max_dt);
}

pm_trajectory_result_t pm_trajectory_start_stop(pm_trajectory_t *trajectory,
		float current_position, float current_velocity, float max_deceleration,
		float max_dt) {
	if (trajectory == NULL || !isfinite(current_position) ||
			!isfinite(current_velocity) ||
			!finite_positive(max_deceleration) || !finite_positive(max_dt)) {
		return PM_TRAJECTORY_RESULT_INVALID_ARGUMENT;
	}
	pm_trajectory_t candidate;
	memset(&candidate, 0, sizeof(candidate));
	candidate.position = current_position;
	candidate.velocity = current_velocity;
	candidate.max_deceleration = max_deceleration;
	candidate.max_dt = max_dt;
	if (current_velocity == 0.0f) {
		candidate.target_position = current_position;
		candidate.state = PM_TRAJECTORY_COMPLETE;
		*trajectory = candidate;
		return PM_TRAJECTORY_RESULT_COMPLETE;
	}
	float duration = fabsf(current_velocity) / max_deceleration;
	float end_position = current_position + current_velocity * duration * 0.5f;
	if (!isfinite(duration) || !isfinite(end_position) || duration <= 0.0f) {
		return PM_TRAJECTORY_RESULT_NUMERIC_ERROR;
	}
	candidate.target_position = end_position;
	if (!append_phase(&candidate,
			-current_velocity / fabsf(current_velocity) * max_deceleration,
			duration, 0.0f)) {
		return PM_TRAJECTORY_RESULT_NUMERIC_ERROR;
	}
	candidate.phases[0].end_position = end_position;
	candidate.position = current_position;
	candidate.velocity = current_velocity;
	candidate.state = PM_TRAJECTORY_RUNNING;
	*trajectory = candidate;
	return PM_TRAJECTORY_RESULT_RUNNING;
}

pm_trajectory_result_t pm_trajectory_advance(pm_trajectory_t *trajectory,
		float dt) {
	if (trajectory == NULL) {
		return PM_TRAJECTORY_RESULT_INVALID_ARGUMENT;
	}
	if (!finite_positive(dt) || !finite_positive(trajectory->max_dt) ||
			dt > trajectory->max_dt) {
		return PM_TRAJECTORY_RESULT_INVALID_DT;
	}
	if (trajectory->state == PM_TRAJECTORY_COMPLETE) {
		return PM_TRAJECTORY_RESULT_COMPLETE;
	}
	if (trajectory->state != PM_TRAJECTORY_RUNNING ||
			trajectory->phase_count > PM_TRAJECTORY_MAX_PHASES ||
			trajectory->phase_index >= trajectory->phase_count) {
		return PM_TRAJECTORY_RESULT_INVALID_ARGUMENT;
	}

	float remaining_dt = dt;
	for (uint8_t step = 0; step < PM_TRAJECTORY_MAX_PHASES &&
			remaining_dt > 0.0f &&
			trajectory->state == PM_TRAJECTORY_RUNNING; step++) {
		pm_trajectory_phase_t *phase =
				&trajectory->phases[trajectory->phase_index];
		float phase_remaining = phase->duration - trajectory->phase_elapsed;
		if (!finite_positive(phase_remaining)) {
			return PM_TRAJECTORY_RESULT_NUMERIC_ERROR;
		}
		if (remaining_dt >= phase_remaining ||
				trajectory->phase_elapsed + remaining_dt >= phase->duration) {
			remaining_dt = remaining_dt > phase_remaining ?
					remaining_dt - phase_remaining : 0.0f;
			trajectory->position = phase->end_position;
			trajectory->velocity = phase->end_velocity;
			trajectory->phase_index++;
			trajectory->phase_elapsed = 0.0f;
			if (trajectory->phase_index == trajectory->phase_count) {
				trajectory->position = trajectory->target_position;
				trajectory->velocity = 0.0f;
				trajectory->state = PM_TRAJECTORY_COMPLETE;
			}
		} else {
			float elapsed = remaining_dt;
			float next_position = trajectory->position +
					trajectory->velocity * elapsed +
					0.5f * phase->acceleration * elapsed * elapsed;
			float next_velocity = trajectory->velocity +
					phase->acceleration * elapsed;
			if (!isfinite(next_position) || !isfinite(next_velocity)) {
				return PM_TRAJECTORY_RESULT_NUMERIC_ERROR;
			}
			trajectory->position = next_position;
			trajectory->velocity = next_velocity;
			trajectory->phase_elapsed += elapsed;
			remaining_dt = 0.0f;
		}
	}
	return trajectory->state == PM_TRAJECTORY_COMPLETE ?
			PM_TRAJECTORY_RESULT_COMPLETE : PM_TRAJECTORY_RESULT_RUNNING;
}