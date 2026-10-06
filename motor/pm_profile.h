#ifndef PM_PROFILE_H
#define PM_PROFILE_H
#include "pm_trajectory.h"
#include <stdbool.h>

/* Absolute counts never pass through floating point. Local profiles are bounded
 * to 65536 counts (including braking excursions), with <= 1 count resolution.
 * Larger moves must be segmented by the caller or are rejected atomically. */
#define PM_PROFILE_SPAN 65536
typedef struct {
	pm_trajectory_t local;
	int64_t origin, endpoint;
	double elapsed;
} pm_profile_t;
bool pm_profile_start(pm_profile_t *p, int64_t position, float velocity, int64_t target,
                      float speed, float acceleration, float deceleration, float max_dt);
bool pm_profile_retarget(pm_profile_t *p, int64_t target, float speed, float acceleration,
                         float deceleration, float max_dt);
bool pm_profile_stop(pm_profile_t *p, int64_t position, float velocity,
                     float deceleration, float max_dt);
bool pm_profile_step(pm_profile_t *p, float dt, int64_t *position, float *velocity);
bool pm_count_add(int64_t a, int64_t b, int64_t *out);
bool pm_count_sub(int64_t a, int64_t b, int64_t *out);
#endif
