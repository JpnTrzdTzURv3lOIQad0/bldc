#include "pm_profile.h"
#include <limits.h>
#include <math.h>
#include <stddef.h>

bool pm_count_add(int64_t a, int64_t b, int64_t *out) {
	if (!out || (b > 0 && a > INT64_MAX - b) || (b < 0 && a < INT64_MIN - b)) return false;
	*out = a + b;
	return true;
}
bool pm_count_sub(int64_t a, int64_t b, int64_t *out) {
	if (!out || (b > 0 && a < INT64_MIN + b) || (b < 0 && a > INT64_MAX + b)) return false;
	*out = a - b;
	return true;
}
static bool bounded(const pm_profile_t *p) {
	for (unsigned i = 0; i < p->local.phase_count; ++i) {
		const pm_trajectory_phase_t *q = &p->local.phases[i];
		if (!isfinite(q->end_position) || fabsf(q->end_position) > PM_PROFILE_SPAN) return false;
		int64_t unused;
		if (!pm_count_add(p->origin, (int64_t)llroundf(q->end_position), &unused)) return false;
	}
	return true;
}
static bool start_local(pm_profile_t *p, int64_t origin, float fraction, float velocity,
		int64_t target, float speed, float accel, float decel, float max_dt) {
	int64_t delta;
	if (!p || !pm_count_sub(target, origin, &delta) ||
			delta < -PM_PROFILE_SPAN || delta > PM_PROFILE_SPAN) return false;
	pm_profile_t next = {0};
	next.origin = origin;
	next.endpoint = target;
	if (pm_trajectory_start(&next.local, fraction, velocity, (float)delta,
			speed, accel, decel, max_dt) > PM_TRAJECTORY_RESULT_COMPLETE || !bounded(&next)) return false;
	*p = next;
	return true;
}
bool pm_profile_start(pm_profile_t *p, int64_t position, float velocity,
		int64_t target, float speed, float accel, float decel, float max_dt) {
	return start_local(p, position, 0.0f, velocity, target, speed, accel, decel, max_dt);
}
bool pm_profile_retarget(pm_profile_t *p, int64_t target, float speed,
		float accel, float decel, float max_dt) {
	if (!p || p->local.state == PM_TRAJECTORY_INACTIVE) return false;
	int64_t integral = (int64_t)llroundf(p->local.position), origin;
	if (!pm_count_add(p->origin, integral, &origin)) return false;
	return start_local(p, origin, p->local.position - (float)integral,
			p->local.velocity, target, speed, accel, decel, max_dt);
}
bool pm_profile_stop(pm_profile_t *p, int64_t position, float velocity,
		float decel, float max_dt) {
	if (!p) return false;
	pm_profile_t next = {0};
	next.origin = position;
	if (pm_trajectory_start_stop(&next.local, 0.0f, velocity, decel, max_dt) >
			PM_TRAJECTORY_RESULT_COMPLETE || !bounded(&next) ||
			!pm_count_add(position, (int64_t)llroundf(next.local.target_position), &next.endpoint)) return false;
	*p = next;
	return true;
}
bool pm_profile_step(pm_profile_t *p, float dt, int64_t *position, float *velocity) {
	if (!p || !position || !velocity || !isfinite(dt) || dt <= 0.0f ||
			dt > p->local.max_dt || p->local.state == PM_TRAJECTORY_INACTIVE) return false;
	pm_profile_t next = *p;
	next.elapsed += (double)dt;
	while (next.local.state == PM_TRAJECTORY_RUNNING) {
		pm_trajectory_phase_t *q = &next.local.phases[next.local.phase_index];
		if (next.elapsed >= (double)q->duration) {
			next.elapsed -= (double)q->duration;
			if (++next.local.phase_index == next.local.phase_count) next.local.state = PM_TRAJECTORY_COMPLETE;
		} else {
			/* Evaluate from the phase boundary, avoiding cumulative position drift. */
			double t = (double)q->duration - next.elapsed;
			next.local.position = (float)((double)q->end_position -
					(double)q->end_velocity * t + (double)0.5f * (double)q->acceleration * t * t);
			next.local.velocity = (float)((double)q->end_velocity - (double)q->acceleration * t);
			break;
		}
	}
	if (next.local.state == PM_TRAJECTORY_COMPLETE) {
		next.local.position = next.local.target_position;
		next.local.velocity = 0.0f;
		*position = next.endpoint;
	} else if (!isfinite(next.local.position) || fabsf(next.local.position) > PM_PROFILE_SPAN ||
			!pm_count_add(next.origin, (int64_t)llroundf(next.local.position), position)) return false;
	*velocity = next.local.velocity;
	*p = next;
	return true;
}
