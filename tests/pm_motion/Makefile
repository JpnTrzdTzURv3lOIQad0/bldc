WHEREAMI := $(dir $(lastword $(MAKEFILE_LIST)))
HERE     := $(realpath $(WHEREAMI)/)
TOP      := $(realpath $(WHEREAMI)/../../)
include $(TOP)/make/unittest-defs.mk

EXTRAINCDIRS += $(TOP)/
CFLAGS += -O0 -Wall -Wdouble-promotion -Werror -g
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS)) -I.
CONLYFLAGS += -std=gnu99

SRC := $(TOP)/motor/pm_interface.c $(TOP)/motor/pm_trajectory.c \
	$(TOP)/motor/pm_feedback.c $(TOP)/motor/pm_control.c

include $(TOP)/make/unittest.mk
