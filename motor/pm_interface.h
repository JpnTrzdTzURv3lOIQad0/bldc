#ifndef PM_INTERFACE_H
#define PM_INTERFACE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	PM_LIFECYCLE_DISABLED = 0,
	PM_LIFECYCLE_UNREFERENCED,
	PM_LIFECYCLE_READY,
	PM_LIFECYCLE_HOMING,
	PM_LIFECYCLE_MOVING,
	PM_LIFECYCLE_SETTLING,
	PM_LIFECYCLE_HOLDING,
	PM_LIFECYCLE_STOPPING,
	PM_LIFECYCLE_FAULTED
} pm_lifecycle_t;

typedef enum {
	PM_COMMAND_ENABLE = 0,
	PM_COMMAND_DISABLE,
	PM_COMMAND_SET_REFERENCE,
	PM_COMMAND_HOME,
	PM_COMMAND_MOVE_ABSOLUTE,
	PM_COMMAND_MOVE_RELATIVE,
	PM_COMMAND_VELOCITY,
	PM_COMMAND_STOP_DECELERATED,
	PM_COMMAND_ABORT_RELEASE,
	PM_COMMAND_CLEAR_FAULT
} pm_command_type_t;

typedef enum {
	PM_RESULT_ACCEPTED_PENDING = 0,
	PM_RESULT_BUSY,
	PM_RESULT_INVALID,
	PM_RESULT_STALE_OWNER,
	PM_RESULT_NOT_READY,
	PM_RESULT_UNSUPPORTED
} pm_result_t;

typedef enum {
	PM_TERMINAL_NONE = 0,
	PM_TERMINAL_COMPLETED,
	PM_TERMINAL_CANCELLED,
	PM_TERMINAL_SUPERSEDED,
	PM_TERMINAL_ABORTED,
	PM_TERMINAL_FAULTED,
	PM_TERMINAL_REJECTED
} pm_terminal_result_t;

typedef struct {
	uint16_t axis_id;
	uint32_t owner_session;
	uint32_t owner_generation;
	uint32_t command_id;
	pm_command_type_t type;
	bool replace_active;
	int64_t value_ticks;
	bool fault_inputs_clear;
	bool feedback_valid;
	bool acknowledged;
} pm_command_t;

typedef struct {
	uint16_t axis_id;
	uint32_t owner_session;
	uint32_t owner_generation;
	pm_lifecycle_t lifecycle;
	bool enabled;
	bool referenced;
	bool command_pending;
	bool command_active;
	uint32_t pending_command_id;
	uint32_t active_command_id;
	pm_command_type_t active_command_type;
	bool active_has_endpoint;
	int64_t active_endpoint_ticks;
	int64_t last_commanded_endpoint_ticks;
	bool has_commanded_endpoint;
	pm_terminal_result_t last_terminal_result;
	uint32_t last_terminal_command_id;
	uint32_t fault_code;
} pm_axis_status_t;

typedef struct {
	pm_axis_status_t status;
	pm_command_t pending_command;
} pm_axis_t;

void pm_axis_init(pm_axis_t *axis, uint16_t axis_id);
pm_result_t pm_axis_claim_owner(pm_axis_t *axis, uint32_t owner_session,
		uint32_t *owner_generation);
void pm_axis_revoke_owner(pm_axis_t *axis, bool invalidate_reference);
pm_result_t pm_axis_submit(pm_axis_t *axis, const pm_command_t *command);
void pm_axis_tick(pm_axis_t *axis);
bool pm_axis_finish(pm_axis_t *axis, uint32_t command_id,
		pm_terminal_result_t result);
void pm_axis_latch_fault(pm_axis_t *axis, uint32_t fault_code);
void pm_axis_get_status(const pm_axis_t *axis, pm_axis_status_t *status);

#endif