#ifndef PM_BEHAVIOR_H
#define PM_BEHAVIOR_H
#include "pm_interface.h"
#include "pm_input.h"

enum { PM_CAP_POSITION = 1U, PM_CAP_VELOCITY = 2U, PM_CAP_CURRENT = 4U,
	PM_CAP_HOME_SWITCH = 8U, PM_CAP_FOLLOW = 16U, PM_CAP_PRESET = 32U,
	PM_CAP_TELEMETRY = 64U, PM_CAP_TORQUE_ESTIMATE = 128U };
typedef enum {
	PM_EX_ABS2, PM_EX_ABS4_HARDSTOP, PM_EX_ABS16_HARDSTOP,
	PM_EX_DIGITAL_POSITION, PM_EX_DIGITAL_TORQUE, PM_EX_DIGITAL_VELOCITY,
	PM_EX_VARIABLE_TORQUE, PM_EX_MANUAL_VELOCITY, PM_EX_INCREMENT2,
	PM_EX_INCREMENT4_HARDSTOP, PM_EX_PULSE_BURST, PM_EX_SELECTED_VELOCITY,
	PM_EX_ABSOLUTE, PM_EX_RELATIVE, PM_EX_VELOCITY, PM_EX_DUAL_AXIS,
	PM_EX_USER_HOME_HARDSTOP, PM_EX_STATUS, PM_EX_ASG_TORQUE, PM_EX_SPEED,
	PM_EX_STEP_DIRECTION, PM_EX_ENCODER_FOLLOW, PM_EX_COUNT
} pm_example_t;
typedef enum { PM_UNSUPPORTED_NONE, PM_UNSUPPORTED_HARDWARE,
	PM_UNSUPPORTED_HARDSTOP, PM_UNSUPPORTED_DUAL_AXIS,
	PM_UNSUPPORTED_TORQUE_CALIBRATION, PM_UNSUPPORTED_CAPABILITY } pm_unsupported_t;
typedef enum { PM_BEHAVIOR_ABSOLUTE, PM_BEHAVIOR_RELATIVE, PM_BEHAVIOR_VELOCITY,
	PM_BEHAVIOR_CURRENT, PM_BEHAVIOR_PRESET, PM_BEHAVIOR_INCREMENT,
	PM_BEHAVIOR_SELECTED_VELOCITY, PM_BEHAVIOR_MANUAL_VELOCITY,
	PM_BEHAVIOR_FOLLOW_POSITION, PM_BEHAVIOR_FOLLOW_VELOCITY,
	PM_BEHAVIOR_PULSE_BURST } pm_behavior_type_t;
typedef struct {
	int64_t positions[16], increments[4], velocities[4];
	uint8_t position_count, increment_count, velocity_count;
	float primary_speed, alternate_speed;
} pm_behavior_config_t;
typedef struct {
	pm_behavior_type_t type;
	int64_t value;
	uint32_t repetitions;
	uint8_t selection;
	bool alternate_speed, frozen;
} pm_behavior_request_t;
pm_unsupported_t pm_behavior_example(pm_example_t example, uint32_t capabilities);
pm_result_t pm_behavior_map(const pm_behavior_config_t *config, uint32_t capabilities,
		const pm_behavior_request_t *request, const pm_axis_status_t *status,
		pm_command_t *command, float *speed_override);
#endif
