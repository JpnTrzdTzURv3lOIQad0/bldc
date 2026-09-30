#ifndef PM_TRAJECTORY_H
#define PM_TRAJECTORY_H

#include <stdint.h>

#define PM_TRAJECTORY_MAX_PHASES 5

typedef enum {
	PM_TRAJECTORY_INACTIVE = 0,
	PM_TRAJECTORY_RUNNING,
	PM_TRAJECTORY_COMPLETE
} pm_trajectory_state_t;

typedef enum {
	PM_TRAJECTORY_RESULT_RUNNING = 0,
	PM_TRAJECTORY_RESULT_COMPLETE,
	PM_TRAJECTORY_RESULT_INVALID_ARGUMENT,
	PM_TRAJECTORY_RESULT_INVALID_DT,
	PM_TRAJECTORY_RESULT_NUMERIC_ERROR
} pm_trajectory_result_t;

typedef struct {
	float duration;
	float acceleration;
	float end_position;
	float end_velocity;
} pm_trajectory_phase_t;

typedef struct {
	pm_trajectory_state_t state;
	float position;
	float velocity;
	float target_position;
	float max_velocity;
	float max_acceleration;
	float max_deceleration;
	float max_dt;
	float phase_elapsed;
	uint8_t phase_count;
	uint8_t phase_index;
	pm_trajectory_phase_t phases[PM_TRAJECTORY_MAX_PHASES];
} pm_trajectory_t;

pm_trajectory_result_t pm_trajectory_start(pm_trajectory_t *trajectory,
		float current_position, float current_velocity, float target_position,
		float max_velocity, float max_acceleration, float max_deceleration,
		float max_dt);
pm_trajectory_result_t pm_trajectory_retarget(pm_trajectory_t *trajectory,
		float target_position, float max_velocity, float max_acceleration,
		float max_deceleration, float max_dt);
pm_trajectory_result_t pm_trajectory_start_stop(pm_trajectory_t *trajectory,
		float current_position, float current_velocity, float max_deceleration,
		float max_dt);
pm_trajectory_result_t pm_trajectory_advance(pm_trajectory_t *trajectory,
		float dt);

#endif