CSRC += \
	motor/foc_math.c \
	motor/mc_interface.c \
	motor/mcpwm.c \
	motor/mcpwm_foc.c \
	motor/virtual_motor.c

PM_INTERFACE_ENABLE ?= 0
ifeq ($(PM_INTERFACE_ENABLE),1)
CSRC += \
	motor/pm_interface.c \
	motor/pm_trajectory.c \
	motor/pm_feedback.c \
	motor/pm_control.c \
	motor/pm_profile.c motor/pm_input.c motor/pm_behavior.c \
	motor/pm_runtime.c motor/pm_protocol.c motor/pm_firmware.c
endif

INCDIR += motor

