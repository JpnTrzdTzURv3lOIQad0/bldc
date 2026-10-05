#include "pm_behavior.h"
#include "pm_profile.h"
#include <limits.h>
#include <math.h>
#include <stddef.h>

pm_unsupported_t pm_behavior_example(pm_example_t e, uint32_t caps) {
	static const uint32_t required[PM_EX_COUNT] = {
		PM_CAP_PRESET | PM_CAP_POSITION | PM_CAP_HOME_SWITCH, 0, 0,
		PM_CAP_POSITION | PM_CAP_FOLLOW, PM_CAP_CURRENT, PM_CAP_VELOCITY,
		PM_CAP_VELOCITY | PM_CAP_CURRENT, PM_CAP_VELOCITY,
		PM_CAP_PRESET | PM_CAP_POSITION | PM_CAP_HOME_SWITCH, 0,
		PM_CAP_POSITION | PM_CAP_PRESET, PM_CAP_VELOCITY | PM_CAP_PRESET,
		PM_CAP_POSITION, PM_CAP_POSITION, PM_CAP_VELOCITY, 0, 0,
		PM_CAP_TELEMETRY, PM_CAP_TELEMETRY | PM_CAP_TORQUE_ESTIMATE,
		PM_CAP_TELEMETRY, PM_CAP_POSITION,
		PM_CAP_POSITION | PM_CAP_VELOCITY | PM_CAP_FOLLOW
	};
	if ((unsigned)e >= PM_EX_COUNT) return PM_UNSUPPORTED_CAPABILITY;
	if (e == PM_EX_ABS4_HARDSTOP || e == PM_EX_ABS16_HARDSTOP ||
			e == PM_EX_INCREMENT4_HARDSTOP || e == PM_EX_USER_HOME_HARDSTOP) return PM_UNSUPPORTED_HARDSTOP;
	if (e == PM_EX_DUAL_AXIS) return PM_UNSUPPORTED_DUAL_AXIS;
	if (e == PM_EX_ASG_TORQUE && !(caps & PM_CAP_TORQUE_ESTIMATE)) return PM_UNSUPPORTED_TORQUE_CALIBRATION;
	return (caps & required[e]) == required[e] ? PM_UNSUPPORTED_NONE : PM_UNSUPPORTED_HARDWARE;
}
pm_result_t pm_behavior_map(const pm_behavior_config_t *c, uint32_t caps,
		const pm_behavior_request_t *r, const pm_axis_status_t *s,
		pm_command_t *out, float *speed) {
	if (!c || !r || !s || !out || !speed || c->position_count > 16 ||
			c->increment_count > 4 || c->velocity_count > 4) return PM_RESULT_INVALID;
	pm_command_t cmd = *out;
	cmd.value_ticks = r->value;
	*speed = 0.0f;
	uint32_t required = 0;
	switch (r->type) {
	case PM_BEHAVIOR_ABSOLUTE: cmd.type = PM_COMMAND_MOVE_ABSOLUTE; required = PM_CAP_POSITION; break;
	case PM_BEHAVIOR_RELATIVE: cmd.type = PM_COMMAND_MOVE_RELATIVE; required = PM_CAP_POSITION; break;
	case PM_BEHAVIOR_VELOCITY: cmd.type = PM_COMMAND_VELOCITY; required = PM_CAP_VELOCITY; break;
	case PM_BEHAVIOR_CURRENT: cmd.type = PM_COMMAND_CURRENT; required = PM_CAP_CURRENT; break;
	case PM_BEHAVIOR_PRESET:
		if (r->selection >= c->position_count) return PM_RESULT_INVALID;
		cmd.type = PM_COMMAND_MOVE_ABSOLUTE; cmd.value_ticks = c->positions[r->selection];
		required = PM_CAP_POSITION | PM_CAP_PRESET; break;
	case PM_BEHAVIOR_INCREMENT: {
		if (r->selection >= c->increment_count || !r->repetitions) return PM_RESULT_INVALID;
		int64_t inc = c->increments[r->selection];
		if ((inc > 0 && inc > INT64_MAX / r->repetitions) ||
				(inc < 0 && inc < INT64_MIN / r->repetitions)) return PM_RESULT_INVALID;
		cmd.type = PM_COMMAND_MOVE_RELATIVE; cmd.value_ticks = inc * r->repetitions;
		required = PM_CAP_POSITION | PM_CAP_PRESET; break;
	}
	case PM_BEHAVIOR_SELECTED_VELOCITY:
		if (r->selection >= c->velocity_count) return PM_RESULT_INVALID;
		cmd.type = PM_COMMAND_VELOCITY; cmd.value_ticks = c->velocities[r->selection];
		required = PM_CAP_VELOCITY | PM_CAP_PRESET; break;
	case PM_BEHAVIOR_MANUAL_VELOCITY:
		if (!pm_count_add(s->command_active && s->active_command_type == PM_COMMAND_VELOCITY ?
				s->active_value : 0, r->value, &cmd.value_ticks)) return PM_RESULT_INVALID;
		cmd.type = PM_COMMAND_VELOCITY; required = PM_CAP_VELOCITY; break;
	case PM_BEHAVIOR_FOLLOW_POSITION:
	case PM_BEHAVIOR_FOLLOW_VELOCITY:
		if (r->frozen) return PM_RESULT_BUSY;
		cmd.type = r->type == PM_BEHAVIOR_FOLLOW_POSITION ? PM_COMMAND_MOVE_ABSOLUTE : PM_COMMAND_VELOCITY;
		required = PM_CAP_FOLLOW | (r->type == PM_BEHAVIOR_FOLLOW_POSITION ? PM_CAP_POSITION : PM_CAP_VELOCITY); break;
	case PM_BEHAVIOR_PULSE_BURST:
		*speed = r->alternate_speed ? c->alternate_speed : c->primary_speed;
		if (!isfinite(*speed) || *speed <= 0.0f) return PM_RESULT_INVALID;
		cmd.type = PM_COMMAND_MOVE_RELATIVE; required = PM_CAP_POSITION | PM_CAP_PRESET; break;
	default: return PM_RESULT_UNSUPPORTED;
	}
	if ((caps & required) != required) return PM_RESULT_UNSUPPORTED;
	cmd.replace_active = s->command_active;
	*out = cmd;
	return PM_RESULT_ACCEPTED_PENDING;
}
