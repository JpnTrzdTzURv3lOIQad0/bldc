#ifndef PM_INPUT_H
#define PM_INPUT_H
#include <stdbool.h>
#include <stdint.h>

typedef enum { PM_INPUT_ANALOG, PM_INPUT_DIGITAL, PM_INPUT_SELECTOR,
	PM_INPUT_TRIGGER, PM_INPUT_QUADRATURE, PM_INPUT_INDEX, PM_INPUT_HOME,
	PM_INPUT_LIMIT } pm_input_type_t;
typedef struct {
	pm_input_type_t type;
	uint16_t source_id;
	uint32_t max_age_us, debounce_us;
	int64_t minimum, maximum;
	bool inverted;
	/* Positive rational scale; both zero selects identity. Applied after polarity. */
	uint32_t scale_numerator, scale_denominator;
	uint32_t filter_time_constant_us; /* Optional analog low-pass; zero bypasses. */
} pm_input_config_t;
typedef struct {
	uint16_t source_id;
	uint32_t owner_session, owner_generation, sequence, acquired_us;
	int64_t value;
	bool healthy;
} pm_input_event_t;
typedef struct {
	pm_input_config_t config;
	pm_input_event_t latest;
	int64_t value, candidate;
	uint32_t candidate_since_us;
	bool initialized, valid;
	double filtered;
} pm_input_t;
bool pm_input_init(pm_input_t *input, const pm_input_config_t *config);
bool pm_input_update(pm_input_t *input, const pm_input_event_t *event,
		uint32_t session, uint32_t generation, uint32_t now_us);
bool pm_input_fresh(const pm_input_t *input, uint32_t now_us);
#endif
