#include "pm_input.h"
#include <limits.h>
#include <math.h>
#include <string.h>

bool pm_input_init(pm_input_t *p, const pm_input_config_t *c) {
	if (!p || !c || (unsigned)c->type > PM_INPUT_LIMIT || !c->max_age_us ||
			c->max_age_us > INT32_MAX || c->debounce_us > c->max_age_us ||
			c->minimum > c->maximum || (c->type == PM_INPUT_TRIGGER && c->debounce_us) ||
			((c->scale_numerator == 0) != (c->scale_denominator == 0)) ||
			(c->filter_time_constant_us && (c->type != PM_INPUT_ANALOG || c->filter_time_constant_us > INT32_MAX))) return false;
	memset(p, 0, sizeof(*p));
	p->config = *c;
	return true;
}
bool pm_input_fresh(const pm_input_t *p, uint32_t now) {
	return p && p->valid && now - p->latest.acquired_us <= p->config.max_age_us;
}
bool pm_input_update(pm_input_t *p, const pm_input_event_t *e,
		uint32_t session, uint32_t generation, uint32_t now) {
	if (!p || !e || e->source_id != p->config.source_id || !session || !generation ||
			e->owner_session != session || e->owner_generation != generation) return false;
	if (!e->healthy || now - e->acquired_us > p->config.max_age_us ||
			e->value < p->config.minimum || e->value > p->config.maximum ||
			(p->config.inverted && e->value == INT64_MIN)) {
		p->valid = false;
		return false;
	}
	if (p->initialized && (e->sequence - p->latest.sequence == 0 ||
			e->sequence - p->latest.sequence > INT32_MAX ||
			e->acquired_us - p->latest.acquired_us > INT32_MAX)) return false;
	int64_t value = e->value;
	if (p->config.inverted) {
		if (p->config.type == PM_INPUT_DIGITAL || p->config.type == PM_INPUT_HOME ||
				p->config.type == PM_INPUT_LIMIT) value = !value;
		else value = -value;
	}
	if (p->config.scale_numerator) {
		int64_t n = p->config.scale_numerator;
		if ((value > 0 && value > INT64_MAX / n) || (value < 0 && value < INT64_MIN / n)) {
			p->valid = false; return false;
		}
		value = value * n / p->config.scale_denominator;
	}
	if (p->config.filter_time_constant_us) {
		if (value < -(INT64_C(1) << 40) || value > (INT64_C(1) << 40)) { p->valid = false; return false; }
		if (!p->initialized) p->filtered = (double)value;
		else {
			double dt = (double)(e->acquired_us - p->latest.acquired_us);
			p->filtered += dt / ((double)p->config.filter_time_constant_us + dt) * ((double)value - p->filtered);
		}
		value = (int64_t)llround(p->filtered);
	}
	if (!p->initialized || value != p->candidate) {
		p->candidate = value;
		p->candidate_since_us = e->acquired_us;
	}
	p->latest = *e;
	p->initialized = true;
	if (e->acquired_us - p->candidate_since_us >= p->config.debounce_us) {
		p->value = value;
		p->valid = true;
	}
	return p->valid;
}
