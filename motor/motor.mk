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
	motor/pm_control.c
endif

INCDIR += motor

