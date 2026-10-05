#ifndef PM_RUNTIME_H
#define PM_RUNTIME_H
#include <stddef.h>
#include "pm_behavior.h"
#include "pm_control.h"
#include "pm_feedback.h"
#include "pm_profile.h"

#define PM_REPLAY_SLOTS 8
enum { PM_FAULT_FEEDBACK = 2, PM_FAULT_TIMING, PM_FAULT_BOUNDS,
	PM_FAULT_HOME, PM_FAULT_BACKEND, PM_FAULT_FOLLOWING, PM_FAULT_OUTPUT_EXPIRED };
typedef struct {
	bool qualified;
	uint16_t axis_id;
	uint32_t capabilities, lease_max_us, output_max_age_us, source_max_age_us;
	int64_t minimum_position, maximum_position;
	float acceleration, deceleration;
	pm_feedback_config_t feedback;
	pm_control_config_t control;
	pm_settle_config_t settle;
	pm_behavior_config_t behavior;
	struct {
		float seek_velocity, approach_velocity, current_limit_amps;
		int64_t max_travel, reference, offset;
		uint32_t timeout_us;
	} home;
} pm_runtime_config_t;
typedef struct {
	pm_feedback_sample_t feedback;
	bool has_sample, home_active, home_valid, positive_limit, negative_limit;
	bool fault_clear;
	float measured_current_amps;
} pm_runtime_sample_t;
typedef struct {
	pm_axis_status_t axis;
	pm_feedback_status_t feedback;
	pm_control_output_t control;
	uint32_t capabilities, epoch;
	bool output_valid;
} pm_runtime_status_t;
typedef enum { PM_HOME_IDLE, PM_HOME_SEEK, PM_HOME_BRAKE_SEEK,
	PM_HOME_BACKOFF, PM_HOME_BRAKE_BACKOFF, PM_HOME_APPROACH,
	PM_HOME_BRAKE_FINAL, PM_HOME_OFFSET } pm_home_state_t;
typedef struct {
	pm_axis_t axis;
	pm_feedback_t feedback;
	pm_control_t controller;
	pm_profile_t profile;
	pm_control_output_t output;
	pm_runtime_sample_t sample;
	int64_t commanded_position, home_start, home_latch_raw;
	float commanded_velocity, speed_limit, current_limit;
	uint32_t tick_us, clock_ms, clock_remainder_us, velocity_stable_us;
	uint32_t home_started_us, source_acquired_us;
	bool clock_started, velocity_stable, following, source_stopping;
	pm_home_state_t home_state;
} pm_engine_t;
typedef uintptr_t (*pm_lock_fn)(void *context);
typedef void (*pm_unlock_fn)(void *context, uintptr_t token);
typedef struct {
	pm_command_t command;
	pm_result_t result;
	bool valid;
} pm_replay_t;
typedef struct {
	pm_runtime_config_t config;
	pm_engine_t engine;
	pm_runtime_status_t status;
	pm_lock_fn lock;
	pm_unlock_fn unlock;
	void *lock_context;
	uint32_t revision, epoch, lease_deadline_us, output_deadline_us, highest_id;
	float demand;
	bool output_valid;
	pm_replay_t replay[PM_REPLAY_SLOTS];
	unsigned replay_next;
} pm_runtime_t;

/* init is called before publishing the object. All other entry points lock.
 * Exactly one scheduler calls tick. Critical sections copy bounded snapshots;
 * profile/servo computation happens outside the lock. No callback may block. */
bool pm_runtime_init(pm_runtime_t *r, const pm_runtime_config_t *config,
		pm_lock_fn lock, pm_unlock_fn unlock, void *context);
pm_result_t pm_runtime_claim(pm_runtime_t *r, uint32_t session, uint32_t lease_us,
		uint32_t now_us, uint32_t *generation);
pm_result_t pm_runtime_renew(pm_runtime_t *r, uint32_t session, uint32_t generation,
		uint32_t lease_us, uint32_t now_us);
pm_result_t pm_runtime_submit(pm_runtime_t *r, const pm_command_t *command, uint32_t now_us);
pm_result_t pm_runtime_behavior(pm_runtime_t *r, const pm_behavior_request_t *request,
		const pm_input_t *input, pm_command_t *command, uint32_t now_us);
void pm_runtime_revoke(pm_runtime_t *r, uint32_t fault_code);
void pm_runtime_tick(pm_runtime_t *r, const pm_runtime_sample_t *sample, uint32_t now_us);
/* Returns true while PM owns the axis, including invalid output (zero demand).
 * Never use demand after a false return. Called at current consumption rate. */
bool pm_runtime_consume(pm_runtime_t *r, uint32_t now_us, float *demand);
void pm_runtime_status(pm_runtime_t *r, pm_runtime_status_t *status);
bool pm_runtime_config_valid(const pm_runtime_config_t *config);
#endif
