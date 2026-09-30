#include "pm_interface.h"

#include <limits.h>
#include <string.h>

static uint32_t next_generation(uint32_t generation) {
	generation++;
	return generation == 0 ? 1 : generation;
}

static bool is_position_command(pm_command_type_t type) {
	return type == PM_COMMAND_MOVE_ABSOLUTE ||
			type == PM_COMMAND_MOVE_RELATIVE;
}

static void record_terminal(pm_axis_status_t *status, uint32_t command_id,
		pm_terminal_result_t result) {
	status->last_terminal_command_id = command_id;
	status->last_terminal_result = result;
}

static void cancel_active(pm_axis_status_t *status,
		pm_terminal_result_t result) {
	if (status->command_active) {
		record_terminal(status, status->active_command_id, result);
		status->command_active = false;
		status->active_has_endpoint = false;
	}
}

static bool add_ticks(int64_t left, int64_t right, int64_t *sum) {
	if ((right > 0 && left > INT64_MAX - right) ||
			(right < 0 && left < INT64_MIN - right)) {
		return false;
	}
	*sum = left + right;
	return true;
}

static bool request_is_valid(const pm_command_t *command) {
	return command->type >= PM_COMMAND_ENABLE &&
			command->type <= PM_COMMAND_CLEAR_FAULT &&
			command->command_id != 0 &&
			command->owner_session != 0 &&
			command->owner_generation != 0;
}

void pm_axis_init(pm_axis_t *axis, uint16_t axis_id) {
	if (axis == NULL) {
		return;
	}
	memset(axis, 0, sizeof(*axis));
	axis->status.axis_id = axis_id;
	axis->status.lifecycle = PM_LIFECYCLE_DISABLED;
}

pm_result_t pm_axis_claim_owner(pm_axis_t *axis, uint32_t owner_session,
		uint32_t *owner_generation) {
	if (axis == NULL || owner_generation == NULL || owner_session == 0) {
		return PM_RESULT_INVALID;
	}
	if (axis->status.owner_session != 0) {
		return PM_RESULT_BUSY;
	}
	axis->status.owner_generation = next_generation(
			axis->status.owner_generation);
	axis->status.owner_session = owner_session;
	*owner_generation = axis->status.owner_generation;
	return PM_RESULT_ACCEPTED_PENDING;
}

void pm_axis_revoke_owner(pm_axis_t *axis, bool invalidate_reference) {
	if (axis == NULL) {
		return;
	}
	cancel_active(&axis->status, PM_TERMINAL_CANCELLED);
	if (axis->status.command_pending) {
		record_terminal(&axis->status, axis->status.pending_command_id,
				PM_TERMINAL_CANCELLED);
	}
	axis->status.command_pending = false;
	axis->status.pending_command_id = 0;
	axis->status.owner_session = 0;
	axis->status.owner_generation = next_generation(
			axis->status.owner_generation);
	axis->status.enabled = false;
	axis->status.lifecycle = PM_LIFECYCLE_DISABLED;
	axis->status.fault_code = 0;
	if (invalidate_reference) {
		axis->status.referenced = false;
		axis->status.has_commanded_endpoint = false;
	}
}

pm_result_t pm_axis_submit(pm_axis_t *axis, const pm_command_t *command) {
	if (axis == NULL || command == NULL || !request_is_valid(command)) {
		return PM_RESULT_INVALID;
	}
	if (command->axis_id != axis->status.axis_id) {
		return PM_RESULT_INVALID;
	}
	if (command->owner_session != axis->status.owner_session ||
			command->owner_generation != axis->status.owner_generation) {
		return PM_RESULT_STALE_OWNER;
	}
	if (axis->status.command_pending) {
		return PM_RESULT_BUSY;
	}
	if (axis->status.command_active) {
		bool control_command = command->type == PM_COMMAND_DISABLE ||
				command->type == PM_COMMAND_ABORT_RELEASE ||
				command->type == PM_COMMAND_STOP_DECELERATED;
		if (!control_command && !command->replace_active) {
			return PM_RESULT_BUSY;
		}
		if (!control_command && (!is_position_command(
				axis->status.active_command_type) ||
				!is_position_command(command->type))) {
			return PM_RESULT_UNSUPPORTED;
		}
	} else if (command->replace_active) {
		return PM_RESULT_INVALID;
	}
	axis->pending_command = *command;
	axis->status.command_pending = true;
	axis->status.pending_command_id = command->command_id;
	return PM_RESULT_ACCEPTED_PENDING;
}

static void reject_pending(pm_axis_t *axis) {
	record_terminal(&axis->status, axis->pending_command.command_id,
			PM_TERMINAL_REJECTED);
	axis->status.command_pending = false;
	axis->status.pending_command_id = 0;
}

static bool activate_motion(pm_axis_t *axis, const pm_command_t *command,
		bool replacing) {
	int64_t endpoint = command->value_ticks;
	if (!axis->status.enabled || !axis->status.referenced) {
		return false;
	}
	if (command->type == PM_COMMAND_MOVE_RELATIVE) {
		int64_t base;
		if (replacing) {
			base = axis->status.active_endpoint_ticks;
		} else if (axis->status.has_commanded_endpoint) {
			base = axis->status.last_commanded_endpoint_ticks;
		} else {
			return false;
		}
		if (!add_ticks(base, command->value_ticks, &endpoint)) {
			return false;
		}
	}
	if (replacing) {
		cancel_active(&axis->status, PM_TERMINAL_SUPERSEDED);
	}
	axis->status.command_active = true;
	axis->status.active_command_id = command->command_id;
	axis->status.active_command_type = command->type;
	axis->status.active_has_endpoint = true;
	axis->status.active_endpoint_ticks = endpoint;
	axis->status.last_commanded_endpoint_ticks = endpoint;
	axis->status.has_commanded_endpoint = true;
	axis->status.lifecycle = PM_LIFECYCLE_MOVING;
	return true;
}

void pm_axis_tick(pm_axis_t *axis) {
	if (axis == NULL || !axis->status.command_pending) {
		return;
	}
	pm_command_t command = axis->pending_command;
	bool replacing = axis->status.command_active && command.replace_active;
	axis->status.command_pending = false;
	axis->status.pending_command_id = 0;
	if (command.owner_session != axis->status.owner_session ||
			command.owner_generation != axis->status.owner_generation) {
		record_terminal(&axis->status, command.command_id, PM_TERMINAL_REJECTED);
		return;
	}

	if (command.type == PM_COMMAND_ENABLE) {
		if (axis->status.lifecycle == PM_LIFECYCLE_FAULTED) {
			reject_pending(axis);
			return;
		}
		axis->status.enabled = true;
		axis->status.lifecycle = axis->status.referenced ?
				PM_LIFECYCLE_READY : PM_LIFECYCLE_UNREFERENCED;
		record_terminal(&axis->status, command.command_id,
				PM_TERMINAL_COMPLETED);
		return;
	}
	if (command.type == PM_COMMAND_DISABLE ||
			command.type == PM_COMMAND_ABORT_RELEASE) {
		cancel_active(&axis->status, command.type == PM_COMMAND_ABORT_RELEASE ?
				PM_TERMINAL_ABORTED : PM_TERMINAL_CANCELLED);
		axis->status.enabled = false;
		axis->status.lifecycle = PM_LIFECYCLE_DISABLED;
		record_terminal(&axis->status, command.command_id,
				command.type == PM_COMMAND_ABORT_RELEASE ?
				PM_TERMINAL_ABORTED : PM_TERMINAL_COMPLETED);
		return;
	}
	if (command.type == PM_COMMAND_SET_REFERENCE) {
		if (axis->status.enabled || axis->status.command_active ||
				axis->status.lifecycle == PM_LIFECYCLE_FAULTED) {
			reject_pending(axis);
			return;
		}
		axis->status.referenced = true;
		axis->status.last_commanded_endpoint_ticks = command.value_ticks;
		axis->status.has_commanded_endpoint = true;
		record_terminal(&axis->status, command.command_id,
				PM_TERMINAL_COMPLETED);
		return;
	}
	if (command.type == PM_COMMAND_CLEAR_FAULT) {
		if (axis->status.lifecycle != PM_LIFECYCLE_FAULTED ||
				!command.fault_inputs_clear || !command.feedback_valid ||
				!command.acknowledged) {
			reject_pending(axis);
			return;
		}
		axis->status.fault_code = 0;
		axis->status.enabled = false;
		axis->status.lifecycle = PM_LIFECYCLE_DISABLED;
		record_terminal(&axis->status, command.command_id,
				PM_TERMINAL_COMPLETED);
		return;
	}
	if (command.type == PM_COMMAND_HOME) {
		if (!axis->status.enabled || axis->status.command_active) {
			reject_pending(axis);
			return;
		}
		axis->status.command_active = true;
		axis->status.active_command_id = command.command_id;
		axis->status.active_command_type = command.type;
		axis->status.active_has_endpoint = false;
		axis->status.lifecycle = PM_LIFECYCLE_HOMING;
		return;
	}
	if (command.type == PM_COMMAND_STOP_DECELERATED) {
		if (!axis->status.enabled) {
			reject_pending(axis);
			return;
		}
		cancel_active(&axis->status, PM_TERMINAL_CANCELLED);
		axis->status.command_active = true;
		axis->status.active_command_id = command.command_id;
		axis->status.active_command_type = command.type;
		axis->status.active_has_endpoint = false;
		axis->status.lifecycle = PM_LIFECYCLE_STOPPING;
		return;
	}
	if (command.type == PM_COMMAND_MOVE_ABSOLUTE ||
			command.type == PM_COMMAND_MOVE_RELATIVE) {
		if (!activate_motion(axis, &command, replacing)) {
			reject_pending(axis);
		}
		return;
	}
	if (command.type == PM_COMMAND_VELOCITY) {
		if (!axis->status.enabled || !axis->status.referenced ||
				axis->status.command_active) {
			reject_pending(axis);
			return;
		}
		axis->status.command_active = true;
		axis->status.active_command_id = command.command_id;
		axis->status.active_command_type = command.type;
		axis->status.active_has_endpoint = false;
		axis->status.lifecycle = PM_LIFECYCLE_MOVING;
		return;
	}
	reject_pending(axis);
}

bool pm_axis_finish(pm_axis_t *axis, uint32_t command_id,
		pm_terminal_result_t result) {
	if (axis == NULL || !axis->status.command_active ||
			axis->status.active_command_id != command_id ||
			result == PM_TERMINAL_NONE || result > PM_TERMINAL_REJECTED) {
		return false;
	}
	bool completed = result == PM_TERMINAL_COMPLETED;
	pm_command_type_t type = axis->status.active_command_type;
	axis->status.command_active = false;
	axis->status.active_has_endpoint = false;
	if (completed && type == PM_COMMAND_HOME) {
		axis->status.referenced = true;
	}
	if (result == PM_TERMINAL_FAULTED) {
		axis->status.enabled = false;
		axis->status.lifecycle = PM_LIFECYCLE_FAULTED;
	} else if (!axis->status.enabled) {
		axis->status.lifecycle = PM_LIFECYCLE_DISABLED;
	} else if (completed && type == PM_COMMAND_STOP_DECELERATED) {
		axis->status.lifecycle = PM_LIFECYCLE_HOLDING;
	} else if (completed && axis->status.referenced) {
		axis->status.lifecycle = PM_LIFECYCLE_HOLDING;
	} else {
		axis->status.lifecycle = axis->status.referenced ?
				PM_LIFECYCLE_READY : PM_LIFECYCLE_UNREFERENCED;
	}
	record_terminal(&axis->status, command_id, result);
	return true;
}

void pm_axis_latch_fault(pm_axis_t *axis, uint32_t fault_code) {
	if (axis == NULL) {
		return;
	}
	cancel_active(&axis->status, PM_TERMINAL_FAULTED);
	if (axis->status.command_pending) {
		record_terminal(&axis->status, axis->status.pending_command_id,
				PM_TERMINAL_REJECTED);
	}
	axis->status.command_pending = false;
	axis->status.pending_command_id = 0;
	axis->status.enabled = false;
	axis->status.lifecycle = PM_LIFECYCLE_FAULTED;
	axis->status.fault_code = fault_code;
}

void pm_axis_get_status(const pm_axis_t *axis, pm_axis_status_t *status) {
	if (axis == NULL || status == NULL) {
		return;
	}
	*status = axis->status;
}