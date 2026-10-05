#include "pm_runtime.h"
#include <limits.h>
#include <math.h>
#include <string.h>

static bool expired(uint32_t now, uint32_t deadline) { return (int32_t)(now - deadline) >= 0; }
static bool positive(float v) { return isfinite(v) && v > 0.0f; }
static bool position_allowed(const pm_runtime_config_t *c, int64_t p) {
	return p >= c->minimum_position && p <= c->maximum_position;
}
static uint32_t caps(const pm_runtime_t *r) {
	return PM_CAP_TELEMETRY | (r->config.qualified ? r->config.capabilities : 0U);
}
bool pm_runtime_config_valid(const pm_runtime_config_t *c) {
	pm_feedback_t f;
	pm_control_t control;
	if (!c || !c->lease_max_us || c->lease_max_us > INT32_MAX ||
			!c->output_max_age_us || c->output_max_age_us > INT32_MAX ||
			!c->source_max_age_us || c->source_max_age_us > INT32_MAX ||
			c->minimum_position >= c->maximum_position || !positive(c->acceleration) ||
			!positive(c->deceleration) || c->axis_id != c->feedback.axis_id ||
			pm_feedback_init(&f, &c->feedback) != PM_FEEDBACK_RESULT_OK ||
			pm_control_init(&control, &c->control) != PM_CONTROL_RESULT_OK ||
			c->control.max_dt_seconds * 1000000.0f > (float)c->output_max_age_us ||
			c->settle.position_tolerance_counts < 0 || !isfinite(c->settle.velocity_tolerance_counts_per_second) ||
			c->settle.velocity_tolerance_counts_per_second < 0 || !c->settle.timeout_ms ||
			c->settle.timeout_ms > INT32_MAX || c->settle.settle_duration_ms > c->settle.timeout_ms ||
			c->control.max_velocity_counts_per_second > 1000000000.0f || c->control.current_limit_amps > 1000000.0f ||
			c->behavior.position_count > 16 || c->behavior.increment_count > 4 || c->behavior.velocity_count > 4)
		return false;
	if (c->capabilities & PM_CAP_HOME_SWITCH) {
		if (!positive(fabsf(c->home.seek_velocity)) || !positive(fabsf(c->home.approach_velocity)) ||
				c->home.seek_velocity * c->home.approach_velocity <= 0.0f ||
				fabsf(c->home.approach_velocity) > fabsf(c->home.seek_velocity) ||
				fabsf(c->home.seek_velocity) > c->control.max_velocity_counts_per_second ||
				!positive(c->home.current_limit_amps) || c->home.current_limit_amps > c->control.current_limit_amps ||
				c->home.max_travel <= 0 || !c->home.timeout_us || c->home.timeout_us > INT32_MAX ||
				!position_allowed(c, c->home.reference) || !position_allowed(c, c->home.offset)) return false;
	}
	return true;
}
static void status_locked(pm_runtime_t *r) {
	r->status.axis = r->engine.axis.status;
	r->status.control = r->engine.output;
	r->status.capabilities = caps(r);
	r->status.epoch = r->epoch;
	r->status.output_valid = r->output_valid;
}
bool pm_runtime_init(pm_runtime_t *r, const pm_runtime_config_t *c,
		pm_lock_fn lock, pm_unlock_fn unlock, void *ctx) {
	if (!r || !lock || !unlock || !pm_runtime_config_valid(c)) return false;
	memset(r, 0, sizeof(*r));
	r->config = *c; r->lock = lock; r->unlock = unlock; r->lock_context = ctx;
	pm_axis_init(&r->engine.axis, c->axis_id);
	pm_feedback_init(&r->engine.feedback, &c->feedback);
	pm_control_init(&r->engine.controller, &c->control);
	r->engine.speed_limit = c->control.max_velocity_counts_per_second;
	r->engine.current_limit = c->control.current_limit_amps;
	r->epoch = 1;
	status_locked(r);
	return true;
}
static void revoke_locked(pm_runtime_t *r, uint32_t fault) {
	if (fault) pm_axis_latch_fault(&r->engine.axis, fault);
	pm_axis_revoke_owner(&r->engine.axis, true);
	r->engine.feedback.referenced = false;
	r->engine.home_state = PM_HOME_IDLE;
	r->engine.following = false;
	r->engine.profile.local.state = PM_TRAJECTORY_INACTIVE;
	r->engine.commanded_velocity = 0.0f;
	pm_control_reset(&r->engine.controller, 0.0f);
	r->output_valid = false; r->demand = 0.0f;
	r->engine.output = (pm_control_output_t){0};
	++r->revision; ++r->epoch;
	status_locked(r);
}
void pm_runtime_revoke(pm_runtime_t *r, uint32_t fault) {
	if (!r) return;
	uintptr_t key = r->lock(r->lock_context);
	revoke_locked(r, fault);
	r->unlock(r->lock_context, key);
}
pm_result_t pm_runtime_claim(pm_runtime_t *r, uint32_t session, uint32_t lease,
		uint32_t now, uint32_t *generation) {
	if (!r || !generation || !session || !lease || lease > r->config.lease_max_us) return PM_RESULT_INVALID;
	uintptr_t key = r->lock(r->lock_context);
	if (r->engine.axis.status.owner_session && expired(now, r->lease_deadline_us)) revoke_locked(r, 0);
	pm_result_t result;
	if (r->engine.axis.status.owner_session == session) {
		*generation = r->engine.axis.status.owner_generation;
		result = PM_RESULT_ACCEPTED_PENDING; /* Retry does not renew lease. */
	} else {
		result = pm_axis_claim_owner(&r->engine.axis, session, generation);
		if (result == PM_RESULT_ACCEPTED_PENDING) {
			r->lease_deadline_us = now + lease; r->highest_id = 0;
			memset(r->replay, 0, sizeof(r->replay));
			++r->revision; ++r->epoch;
		}
	}
	status_locked(r); r->unlock(r->lock_context, key); return result;
}
pm_result_t pm_runtime_renew(pm_runtime_t *r, uint32_t session, uint32_t generation,
		uint32_t lease, uint32_t now) {
	if (!r || !session || !lease || lease > r->config.lease_max_us) return PM_RESULT_INVALID;
	uintptr_t key = r->lock(r->lock_context);
	pm_result_t result = PM_RESULT_STALE_OWNER;
	if (r->engine.axis.status.owner_session && expired(now, r->lease_deadline_us)) revoke_locked(r, 0);
	if (r->engine.axis.status.owner_session == session && r->engine.axis.status.owner_generation == generation) {
		r->lease_deadline_us = now + lease; result = PM_RESULT_ACCEPTED_PENDING;
	}
	r->unlock(r->lock_context, key); return result;
}
static bool same_command(const pm_command_t *a, const pm_command_t *b) {
	return a->axis_id == b->axis_id && a->owner_session == b->owner_session &&
		a->owner_generation == b->owner_generation && a->command_id == b->command_id &&
		a->type == b->type && a->replace_active == b->replace_active && a->value_ticks == b->value_ticks &&
		a->acknowledged == b->acknowledged && a->speed_limit == b->speed_limit &&
		a->current_limit_amps == b->current_limit_amps && a->follow_source == b->follow_source &&
		a->source_acquired_us == b->source_acquired_us;
}
static pm_result_t submit_locked(pm_runtime_t *r, const pm_command_t *cmd, uint32_t now) {
	pm_axis_t *a = &r->engine.axis;
	if (!cmd->command_id || cmd->axis_id != r->config.axis_id || cmd->type > PM_COMMAND_CURRENT ||
			!isfinite(cmd->speed_limit) || cmd->speed_limit < 0 ||
			!isfinite(cmd->current_limit_amps) || cmd->current_limit_amps < 0) return PM_RESULT_INVALID;
	if (a->status.owner_session && expired(now, r->lease_deadline_us)) revoke_locked(r, 0);
	if (!cmd->owner_session || cmd->owner_session != a->status.owner_session ||
			cmd->owner_generation != a->status.owner_generation) return PM_RESULT_STALE_OWNER;
	for (unsigned i = 0; i < PM_REPLAY_SLOTS; ++i) {
		if (r->replay[i].valid && r->replay[i].command.command_id == cmd->command_id)
			return same_command(cmd, &r->replay[i].command) ? r->replay[i].result : PM_RESULT_INVALID;
	}
	if (cmd->command_id <= r->highest_id) return PM_RESULT_INVALID;
	uint32_t needed = 0;
	switch (cmd->type) {
	case PM_COMMAND_MOVE_ABSOLUTE: case PM_COMMAND_MOVE_RELATIVE: needed = PM_CAP_POSITION; break;
	case PM_COMMAND_VELOCITY: needed = PM_CAP_VELOCITY; break;
	case PM_COMMAND_CURRENT: needed = PM_CAP_CURRENT; break;
	case PM_COMMAND_HOME: needed = PM_CAP_HOME_SWITCH; break;
	default: break;
	}
	bool release = cmd->type == PM_COMMAND_DISABLE || cmd->type == PM_COMMAND_ABORT_RELEASE;
	if ((!r->config.qualified && !release) || (caps(r) & needed) != needed) return PM_RESULT_UNSUPPORTED;
	if (cmd->follow_source && (!(caps(r) & PM_CAP_FOLLOW) ||
			now - cmd->source_acquired_us > r->config.source_max_age_us)) return PM_RESULT_NOT_READY;
	if (cmd->speed_limit > r->config.control.max_velocity_counts_per_second ||
			cmd->current_limit_amps > r->config.control.current_limit_amps) return PM_RESULT_INVALID;
	if ((cmd->type == PM_COMMAND_MOVE_ABSOLUTE || cmd->type == PM_COMMAND_SET_REFERENCE) &&
			!position_allowed(&r->config, cmd->value_ticks)) return PM_RESULT_INVALID;
	if (cmd->type == PM_COMMAND_MOVE_ABSOLUTE || cmd->type == PM_COMMAND_MOVE_RELATIVE) {
		int64_t target = cmd->value_ticks, span;
		if (cmd->type == PM_COMMAND_MOVE_RELATIVE &&
				!pm_count_add(a->status.last_commanded_endpoint_ticks, cmd->value_ticks, &target)) return PM_RESULT_INVALID;
		if (!position_allowed(&r->config, target) || !pm_count_sub(target, r->engine.commanded_position, &span) ||
				span > PM_PROFILE_SPAN || span < -PM_PROFILE_SPAN) return PM_RESULT_INVALID;
	}
	if (cmd->type == PM_COMMAND_VELOCITY && fabs((double)cmd->value_ticks) >
			(double)r->config.control.max_velocity_counts_per_second) return PM_RESULT_INVALID;
	if (cmd->type == PM_COMMAND_CURRENT && fabs((double)cmd->value_ticks) >
			(double)r->config.control.current_limit_amps * (double)1000.0f) return PM_RESULT_INVALID;
	pm_result_t result = pm_axis_submit(a, cmd);
	if (result == PM_RESULT_ACCEPTED_PENDING) {
		r->highest_id = cmd->command_id;
		r->replay[r->replay_next] = (pm_replay_t){*cmd, result, true};
		r->replay_next = (r->replay_next + 1) % PM_REPLAY_SLOTS;
		++r->revision;
		if (release) { r->output_valid = false; r->demand = 0.0f; ++r->epoch; }
	}
	status_locked(r); return result;
}
pm_result_t pm_runtime_submit(pm_runtime_t *r, const pm_command_t *cmd, uint32_t now) {
	if (!r || !cmd) return PM_RESULT_INVALID;
	uintptr_t key = r->lock(r->lock_context);
	pm_result_t result = submit_locked(r, cmd, now);
	r->unlock(r->lock_context, key); return result;
}
pm_result_t pm_runtime_behavior(pm_runtime_t *r, const pm_behavior_request_t *request,
		const pm_input_t *input, pm_command_t *cmd, uint32_t now) {
	if (!r || !request || !cmd) return PM_RESULT_INVALID;
	uintptr_t key = r->lock(r->lock_context);
	pm_behavior_request_t req = *request;
	bool follow = req.type == PM_BEHAVIOR_FOLLOW_POSITION || req.type == PM_BEHAVIOR_FOLLOW_VELOCITY;
	pm_result_t result = PM_RESULT_NOT_READY;
	if (!follow || (input && pm_input_fresh(input, now) &&
			input->latest.owner_session == cmd->owner_session && input->latest.owner_generation == cmd->owner_generation)) {
		if (follow) {
			if (input->config.source_id == r->config.feedback.source_id) {
				r->unlock(r->lock_context, key); return PM_RESULT_UNSUPPORTED;
			}
			req.value = req.frozen ? (req.type == PM_BEHAVIOR_FOLLOW_POSITION ?
					r->engine.axis.status.last_commanded_endpoint_ticks : 0) : input->value;
			req.frozen = false;
			cmd->follow_source = true; cmd->source_acquired_us = input->latest.acquired_us;
		}
		float speed;
		result = pm_behavior_map(&r->config.behavior, caps(r), &req, &r->engine.axis.status, cmd, &speed);
		if (result == PM_RESULT_ACCEPTED_PENDING) { cmd->speed_limit = speed; result = submit_locked(r, cmd, now); }
	}
	r->unlock(r->lock_context, key); return result;
}
static float ramp(float value, float target, float rate, float dt) {
	float delta = rate * dt;
	return value < target ? fminf(value + delta, target) : fmaxf(value - delta, target);
}
static void fail(pm_engine_t *e, uint32_t code) {
	if (e->home_state != PM_HOME_IDLE) { e->feedback.referenced = false; e->axis.status.referenced = false; }
	e->home_state = PM_HOME_IDLE; e->following = false;
	e->output = (pm_control_output_t){0}; pm_axis_latch_fault(&e->axis, code);
}
static bool reference(pm_engine_t *e, int64_t position, uint32_t now) {
	if (pm_feedback_set_reference(&e->feedback, position, now) != PM_FEEDBACK_RESULT_OK) return false;
	e->axis.status.referenced = true;
	e->axis.status.last_commanded_endpoint_ticks = position;
	e->axis.status.has_commanded_endpoint = true;
	e->commanded_position = position; e->commanded_velocity = 0.0f;
	e->profile.local.state = PM_TRAJECTORY_INACTIVE;
	pm_control_reset(&e->controller, 0.0f);
	return true;
}
static bool plan_position(pm_engine_t *e, const pm_runtime_config_t *c, int64_t target, bool replace) {
	if (!position_allowed(c, target)) return false;
	bool ok = replace && e->profile.local.state != PM_TRAJECTORY_INACTIVE ?
		pm_profile_retarget(&e->profile, target, e->speed_limit, c->acceleration, c->deceleration, c->control.max_dt_seconds) :
		pm_profile_start(&e->profile, e->commanded_position, e->commanded_velocity,
			target, e->speed_limit, c->acceleration, c->deceleration, c->control.max_dt_seconds);
	if (!ok) return false;
	for (unsigned i = 0; i < e->profile.local.phase_count; ++i) {
		int64_t boundary;
		if (!pm_count_add(e->profile.origin, (int64_t)llroundf(e->profile.local.phases[i].end_position), &boundary) ||
				!position_allowed(c, boundary)) return false;
	}
	return true;
}
static void home_step(pm_engine_t *e, const pm_runtime_config_t *c, uint32_t now, float dt) {
	int64_t distance;
	if (!e->sample.home_valid || now - e->home_started_us >= c->home.timeout_us ||
			!pm_count_sub(e->feedback.raw_position_counts, e->home_start, &distance) ||
			distance > c->home.max_travel || distance < -c->home.max_travel ||
			fabsf(e->sample.measured_current_amps) > c->home.current_limit_amps) { fail(e, PM_FAULT_HOME); return; }
	float target = 0.0f;
	switch (e->home_state) {
	case PM_HOME_SEEK:
		if (e->sample.home_active) e->home_state = PM_HOME_BRAKE_SEEK;
		else target = c->home.seek_velocity;
		break;
	case PM_HOME_BACKOFF:
		if (!e->sample.home_active) e->home_state = PM_HOME_BRAKE_BACKOFF;
		else target = -c->home.approach_velocity;
		break;
	case PM_HOME_APPROACH:
		if (e->sample.home_active) {
			e->home_latch_raw = e->feedback.raw_position_counts;
			e->home_state = PM_HOME_BRAKE_FINAL;
		}
		else target = c->home.approach_velocity;
		break;
	default: break;
	}
	e->commanded_velocity = ramp(e->commanded_velocity, target, c->deceleration, dt);
	bool stopped = e->commanded_velocity == 0.0f &&
		fabsf(e->feedback.velocity_counts_per_second) <= c->settle.velocity_tolerance_counts_per_second;
	if (stopped && e->home_state == PM_HOME_BRAKE_SEEK) e->home_state = PM_HOME_BACKOFF;
	else if (stopped && e->home_state == PM_HOME_BRAKE_BACKOFF) e->home_state = PM_HOME_APPROACH;
	else if (stopped && e->home_state == PM_HOME_BRAKE_FINAL) {
		int64_t after_edge, stopped_coordinate;
		if (!e->sample.home_active ||
				!pm_count_sub(e->feedback.raw_position_counts, e->home_latch_raw, &after_edge) ||
				!pm_count_add(c->home.reference, after_edge, &stopped_coordinate) ||
				!position_allowed(c, stopped_coordinate) || !reference(e, stopped_coordinate, now) ||
				!plan_position(e, c, c->home.offset, false)) { fail(e, PM_FAULT_HOME); return; }
		e->home_state = PM_HOME_OFFSET;
		e->axis.status.active_endpoint_ticks = c->home.offset;
		e->axis.status.active_has_endpoint = true;
	}
}
static void engine_tick(pm_engine_t *e, const pm_runtime_config_t *c,
		const pm_runtime_sample_t *sample, uint32_t now) {
	e->sample = *sample; e->output = (pm_control_output_t){0};
	uint32_t elapsed = e->clock_started ? now - e->tick_us : 0;
	e->clock_started = true; e->tick_us = now;
	if (elapsed > INT32_MAX || (float)elapsed * 0.000001f > c->control.max_dt_seconds) {
		if (e->axis.status.enabled) fail(e, PM_FAULT_TIMING);
		return;
	}
	e->clock_remainder_us += elapsed;
	e->clock_ms += e->clock_remainder_us / 1000U; e->clock_remainder_us %= 1000U;
	float dt = (float)elapsed * 0.000001f;
	if (sample->has_sample) {
		pm_feedback_result_t result = pm_feedback_update(&e->feedback, &sample->feedback, now);
		if (result != PM_FEEDBACK_RESULT_OK && result != PM_FEEDBACK_RESULT_STALE) {
			e->axis.status.referenced = false;
			if (e->axis.status.enabled) { fail(e, PM_FAULT_FEEDBACK); return; }
		}
	}
	pm_feedback_status_t fb;
	pm_feedback_get_status(&e->feedback, now, &fb);
	bool healthy = fb.valid && fb.commutation_valid && sample->fault_clear && isfinite(sample->measured_current_amps);
	if (!fb.referenced) e->axis.status.referenced = false;
	if (!healthy && e->axis.status.enabled) { fail(e, PM_FAULT_FEEDBACK); return; }
	bool has_command = e->axis.priority_pending || e->axis.status.command_pending;
	pm_command_t cmd = e->axis.priority_pending ? e->axis.priority_command : e->axis.pending_command;
	if (has_command) {
		/* Health is sampled here, not trusted from command transport. */
		e->axis.pending_command.fault_inputs_clear = sample->fault_clear;
		e->axis.pending_command.feedback_valid = healthy;
		if ((cmd.type == PM_COMMAND_ENABLE || cmd.type == PM_COMMAND_SET_REFERENCE || cmd.type == PM_COMMAND_HOME) && !healthy) {
			e->axis.status.command_pending = false; e->axis.status.pending_command_id = 0;
			e->axis.status.last_terminal_command_id = cmd.command_id;
			e->axis.status.last_terminal_result = PM_TERMINAL_REJECTED;
		} else {
			pm_axis_tick(&e->axis);
			if ((cmd.type == PM_COMMAND_DISABLE || cmd.type == PM_COMMAND_ABORT_RELEASE) &&
					e->home_state != PM_HOME_IDLE) {
				e->home_state = PM_HOME_IDLE; e->feedback.referenced = false;
				e->axis.status.referenced = false;
			}
			bool active = e->axis.status.command_active && e->axis.status.active_command_id == cmd.command_id;
			bool completed = e->axis.status.last_terminal_command_id == cmd.command_id &&
				e->axis.status.last_terminal_result == PM_TERMINAL_COMPLETED;
			if (cmd.type == PM_COMMAND_SET_REFERENCE && completed && !reference(e, cmd.value_ticks, now)) fail(e, PM_FAULT_FEEDBACK);
			if (cmd.type == PM_COMMAND_ENABLE && completed) {
				e->commanded_position = fb.axis_position_counts; e->commanded_velocity = 0.0f;
				pm_control_reset(&e->controller, fb.velocity_counts_per_second);
			}
			if (active) {
				e->speed_limit = cmd.speed_limit > 0 ? cmd.speed_limit : c->control.max_velocity_counts_per_second;
				e->current_limit = cmd.current_limit_amps > 0 ? cmd.current_limit_amps : c->control.current_limit_amps;
				e->following = cmd.follow_source; e->source_acquired_us = cmd.source_acquired_us;
				e->source_stopping = false;
				e->velocity_stable = false;
				if (cmd.type == PM_COMMAND_MOVE_ABSOLUTE || cmd.type == PM_COMMAND_MOVE_RELATIVE) {
					if (!plan_position(e, c, e->axis.status.active_endpoint_ticks, cmd.replace_active)) fail(e, PM_FAULT_BOUNDS);
				} else if (cmd.type == PM_COMMAND_STOP_DECELERATED) {
					if (e->home_state != PM_HOME_IDLE) { fail(e, PM_FAULT_HOME); return; }
					/* Stop the measured motion, including current-mode motion. */
					if (!pm_profile_stop(&e->profile, fb.axis_position_counts, fb.velocity_counts_per_second,
							c->deceleration, c->control.max_dt_seconds) || !position_allowed(c, e->profile.endpoint)) fail(e, PM_FAULT_BOUNDS);
					else { e->axis.status.active_endpoint_ticks = e->profile.endpoint; e->axis.status.active_has_endpoint = true; }
				} else if (cmd.type == PM_COMMAND_HOME) {
					e->feedback.referenced = false; e->axis.status.referenced = false;
					e->home_start = fb.raw_position_counts; e->home_started_us = now;
					e->home_state = sample->home_active ? PM_HOME_BACKOFF : PM_HOME_SEEK;
				}
			}
		}
	}
	if (!e->axis.status.enabled || e->axis.status.fault_code || !healthy || dt <= 0.0f) return;
	if (e->following && now - e->source_acquired_us > c->source_max_age_us) {
		/* Source loss is latched by ending follow; only an explicit request can resume. */
		e->following = false;
		e->source_stopping = true;
		pm_axis_finish(&e->axis, e->axis.status.active_command_id, PM_TERMINAL_CANCELLED);
		e->axis.status.command_active = true;
		e->axis.status.active_command_type = PM_COMMAND_STOP_DECELERATED;
		e->axis.status.lifecycle = PM_LIFECYCLE_STOPPING;
		if (!pm_profile_stop(&e->profile, fb.axis_position_counts, fb.velocity_counts_per_second,
				c->deceleration, c->control.max_dt_seconds)) { fail(e, PM_FAULT_BOUNDS); return; }
		if (!position_allowed(c, e->profile.endpoint)) { fail(e, PM_FAULT_BOUNDS); return; }
		e->axis.status.active_has_endpoint = true; e->axis.status.active_endpoint_ticks = e->profile.endpoint;
	}
	pm_command_type_t type = e->axis.status.active_command_type;
	bool home = e->axis.status.command_active && type == PM_COMMAND_HOME;
	if (home && e->home_state != PM_HOME_OFFSET) home_step(e, c, now, dt);
	bool position = !e->axis.status.command_active || type == PM_COMMAND_MOVE_ABSOLUTE ||
		type == PM_COMMAND_MOVE_RELATIVE || type == PM_COMMAND_STOP_DECELERATED || (home && e->home_state == PM_HOME_OFFSET);
	if (position && e->axis.status.command_active) {
		if (!pm_profile_step(&e->profile, dt, &e->commanded_position, &e->commanded_velocity)) { fail(e, PM_FAULT_BOUNDS); return; }
		pm_settle_result_t settle = pm_axis_update_settle(&e->axis, e->axis.status.active_command_id,
			e->profile.local.state == PM_TRAJECTORY_COMPLETE, true, fb.axis_position_counts,
			fb.velocity_counts_per_second, e->clock_ms, &c->settle);
		if (settle == PM_SETTLE_RESULT_COMPLETED) {
			if (e->source_stopping) {
				e->axis.status.last_terminal_result = PM_TERMINAL_CANCELLED;
				e->source_stopping = false;
			}
			e->axis.status.last_commanded_endpoint_ticks = e->commanded_position;
			e->axis.status.has_commanded_endpoint = true;
			e->home_state = PM_HOME_IDLE;
		}
	} else if (!home && type == PM_COMMAND_VELOCITY) {
		float target = (float)e->axis.status.active_value;
		float rate = fabsf(target) > fabsf(e->commanded_velocity) && target * e->commanded_velocity >= 0.0f ?
			c->acceleration : c->deceleration;
		e->commanded_velocity = ramp(e->commanded_velocity, target, rate, dt);
		bool at_speed = e->commanded_velocity == target && fabsf(fb.velocity_counts_per_second - target) <= c->settle.velocity_tolerance_counts_per_second;
		if (!at_speed) e->velocity_stable = false;
		else if (!e->velocity_stable) { e->velocity_stable = true; e->velocity_stable_us = now; }
		e->axis.status.at_target_velocity = at_speed && (now - e->velocity_stable_us) / 1000U >= c->settle.settle_duration_ms;
	}
	if (e->axis.status.fault_code) return;
	pm_feedback_get_status(&e->feedback, now, &fb); /* Reference may have changed during HOME. */
	if (fb.referenced && (!position_allowed(c, fb.axis_position_counts) ||
			(position && !position_allowed(c, e->commanded_position)))) { fail(e, PM_FAULT_BOUNDS); return; }
	if (fabsf(fb.velocity_counts_per_second) > c->control.max_velocity_counts_per_second ||
			(sample->positive_limit && (e->commanded_velocity > 0.0f || fb.velocity_counts_per_second > 0.0f)) ||
			(sample->negative_limit && (e->commanded_velocity < 0.0f || fb.velocity_counts_per_second < 0.0f))) { fail(e, PM_FAULT_BOUNDS); return; }
	pm_control_input_t in = {0};
	in.enabled = e->axis.status.enabled; in.feedback_valid = fb.valid;
	in.commutation_valid = fb.commutation_valid; in.referenced = fb.referenced;
	in.commanded_position_counts = e->commanded_position; in.measured_position_counts = fb.axis_position_counts;
	in.commanded_velocity_counts_per_second = e->commanded_velocity;
	in.measured_velocity_counts_per_second = fb.velocity_counts_per_second;
	in.mode = home && e->home_state != PM_HOME_OFFSET ? PM_CONTROL_HOMING : position ? PM_CONTROL_POSITION :
		type == PM_COMMAND_CURRENT ? PM_CONTROL_CURRENT : PM_CONTROL_VELOCITY;
	in.commanded_current_amps = type == PM_COMMAND_CURRENT ? (float)e->axis.status.active_value / 1000.0f : 0.0f;
	if (in.mode == PM_CONTROL_CURRENT && ((sample->positive_limit && in.commanded_current_amps > 0.0f) ||
			(sample->negative_limit && in.commanded_current_amps < 0.0f))) { fail(e, PM_FAULT_BOUNDS); return; }
	e->controller.config.current_limit_amps = home ? c->home.current_limit_amps : e->current_limit;
	pm_control_result_t result = pm_control_step(&e->controller, &in, dt, &e->output);
	if ((sample->positive_limit && e->output.iq_demand_amps > 0.0f) ||
			(sample->negative_limit && e->output.iq_demand_amps < 0.0f)) fail(e, PM_FAULT_BOUNDS);
	else if (e->output.following_error_exceeded) fail(e, PM_FAULT_FOLLOWING);
	else if (result != PM_CONTROL_RESULT_OK && result != PM_CONTROL_RESULT_NOT_READY) fail(e, PM_FAULT_BACKEND);
}
void pm_runtime_tick(pm_runtime_t *r, const pm_runtime_sample_t *sample, uint32_t now) {
	if (!r || !sample) return;
	uintptr_t key = r->lock(r->lock_context);
	if (r->engine.axis.status.owner_session && expired(now, r->lease_deadline_us)) revoke_locked(r, 0);
	pm_engine_t next = r->engine;
	uint32_t revision = r->revision, epoch = r->epoch;
	r->unlock(r->lock_context, key);
	engine_tick(&next, &r->config, sample, now);
	key = r->lock(r->lock_context);
	if (r->revision == revision && r->epoch == epoch) {
		bool new_fault = !r->engine.axis.status.fault_code && next.axis.status.fault_code;
		r->engine = next;
		if (new_fault) {
			revoke_locked(r, 0);
		}
		r->output_valid = r->config.qualified && next.axis.status.owner_session && next.axis.status.enabled &&
			!next.axis.status.fault_code && next.output.output_valid && !expired(now, r->lease_deadline_us);
		r->demand = r->output_valid ? r->engine.output.iq_demand_amps : 0.0f;
		r->output_deadline_us = now + r->config.output_max_age_us;
		pm_feedback_get_status(&r->engine.feedback, now, &r->status.feedback);
		status_locked(r);
	}
	r->unlock(r->lock_context, key);
}
bool pm_runtime_consume(pm_runtime_t *r, uint32_t now, float *demand) {
	if (!r || !demand) return false;
	uintptr_t key = r->lock(r->lock_context);
	bool owned = r->engine.axis.status.owner_session != 0;
	if (owned && expired(now, r->lease_deadline_us)) revoke_locked(r, 0);
	else if (r->output_valid && expired(now, r->output_deadline_us)) revoke_locked(r, PM_FAULT_OUTPUT_EXPIRED);
	*demand = r->output_valid ? r->demand : 0.0f;
	r->unlock(r->lock_context, key); return owned;
}
void pm_runtime_status(pm_runtime_t *r, pm_runtime_status_t *status) {
	if (!r || !status) return;
	uintptr_t key = r->lock(r->lock_context); *status = r->status; r->unlock(r->lock_context, key);
}
