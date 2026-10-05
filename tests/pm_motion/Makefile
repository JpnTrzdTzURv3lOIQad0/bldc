WHEREAMI := $(dir $(lastword $(MAKEFILE_LIST)))
HERE     := $(realpath $(WHEREAMI)/)
TOP      := $(realpath $(WHEREAMI)/../../)
include $(TOP)/make/unittest-defs.mk

EXTRAINCDIRS += $(TOP)/ $(TOP)/util
CFLAGS += -O0 -Wall -Wdouble-promotion -Werror -g -DNO_STM32
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS)) -I.
CONLYFLAGS += -std=gnu99

SRC := $(TOP)/motor/pm_interface.c $(TOP)/motor/pm_trajectory.c \
	$(TOP)/motor/pm_feedback.c $(TOP)/motor/pm_control.c \
	$(TOP)/motor/pm_profile.c $(TOP)/motor/pm_input.c \
	$(TOP)/motor/pm_behavior.c $(TOP)/motor/pm_runtime.c $(TOP)/motor/pm_protocol.c \
	$(TOP)/comm/packet.c $(TOP)/util/crc.c

include $(TOP)/make/unittest.mk
