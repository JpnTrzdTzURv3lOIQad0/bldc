# Include from your Makefile. Apply these flags to YOUR C/C++ objects, not ASM.
# No optimization, ABI, language standard, libc, or linker script is overridden.
QUALITY_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/..)
QUALITY_COMMON = @"$(QUALITY_DIR)/compiler/gcc-common.rsp"
QUALITY_C = $(QUALITY_COMMON) @"$(QUALITY_DIR)/compiler/gcc-c.rsp"
QUALITY_CXX = $(QUALITY_COMMON) @"$(QUALITY_DIR)/compiler/gcc-cxx.rsp"
QUALITY_ANALYZER_C = @"$(QUALITY_DIR)/compiler/gcc-analyzer-c.rsp"

# Set per product after stack analysis; these are warning thresholds, not proof.
ifneq ($(strip $(QUALITY_FRAME_BYTES)),)
QUALITY_COMMON += -Wframe-larger-than=$(QUALITY_FRAME_BYTES)
endif
ifneq ($(strip $(QUALITY_STACK_BYTES)),)
QUALITY_COMMON += -Wstack-usage=$(QUALITY_STACK_BYTES)
endif

# Recommended evidence emitted by a separate non-LTO resource-analysis build.
QUALITY_STACK_REPORT = -fstack-usage
QUALITY_LINK = -Wl,--fatal-warnings,--print-memory-usage

# Append QUALITY_C to C-only flags and QUALITY_CXX to C++-only flags.
# Modernization target: -std=gnu23 on C commands only. Keep a GNU99 comparison job.
# For a NEW C++14 project use -std=gnu++14; preserve existing C++ dialect/ABI.
# Append QUALITY_ANALYZER_C only to a separate C analysis build at -O0, no LTO.
# Keep the ordinary optimized production warning build as an independent gate.
