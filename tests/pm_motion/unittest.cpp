#include "gtest/gtest.h"

extern "C" {
#include "motor/pm_interface.h"
}

class PmMotion : public testing::Test {
protected:
	pm_axis_t axis;
	uint32_t owner_generation;

	void SetUp() override {
		pm_axis_init(&axis, 0);
		owner_generation = 0;
		ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING,
				pm_axis_claim_owner(&axis, 42, &owner_generation));
	}

	pm_command_t command(uint32_t id, pm_command_type_t type,
			int64_t value_ticks = 0, bool replace = false) {
		pm_command_t result = {};
		result.axis_id = 0;
		result.owner_session = 42;
		result.owner_generation = owner_generation;
		result.command_id = id;
		result.type = type;
		result.value_ticks = value_ticks;
		result.replace_active = replace;
		return result;
	}

	void prepare_referenced_axis() {
		pm_command_t set_reference = command(1, PM_COMMAND_SET_REFERENCE);
		ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING,
				pm_axis_submit(&axis, &set_reference));
		pm_axis_tick(&axis);
		pm_command_t enable = command(2, PM_COMMAND_ENABLE);
		ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &enable));
		pm_axis_tick(&axis);
	}
};

TEST_F(PmMotion, StartsDisabledAndUnreferenced) {
	pm_axis_status_t status = {};
	pm_axis_get_status(&axis, &status);
	EXPECT_EQ(PM_LIFECYCLE_DISABLED, status.lifecycle);
	EXPECT_FALSE(status.enabled);
	EXPECT_FALSE(status.referenced);
	EXPECT_EQ(0U, status.axis_id);
}

TEST_F(PmMotion, RejectsWrongAxisAndStaleOwner) {
	pm_command_t request = command(1, PM_COMMAND_ENABLE);
	request.axis_id = 1;
	EXPECT_EQ(PM_RESULT_INVALID, pm_axis_submit(&axis, &request));

	pm_axis_revoke_owner(&axis, false);
	request = command(2, PM_COMMAND_ENABLE);
	EXPECT_EQ(PM_RESULT_STALE_OWNER, pm_axis_submit(&axis, &request));
}

TEST_F(PmMotion, OnePendingSlotReturnsBusy) {
	pm_command_t first = command(1, PM_COMMAND_SET_REFERENCE);
	pm_command_t second = command(2, PM_COMMAND_SET_REFERENCE);
	EXPECT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &first));
	EXPECT_EQ(PM_RESULT_BUSY, pm_axis_submit(&axis, &second));
	pm_axis_tick(&axis);
	EXPECT_EQ(PM_TERMINAL_COMPLETED, axis.status.last_terminal_result);
	EXPECT_EQ(1U, axis.status.last_terminal_command_id);
}

TEST_F(PmMotion, RelativeReplacementUsesSupersededEndpoint) {
	prepare_referenced_axis();
	pm_command_t absolute = command(3, PM_COMMAND_MOVE_ABSOLUTE, 300);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &absolute));
	pm_axis_tick(&axis);

	pm_command_t relative = command(4, PM_COMMAND_MOVE_RELATIVE, 50, true);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &relative));
	EXPECT_EQ(300, axis.status.active_endpoint_ticks);
	pm_axis_tick(&axis);
	EXPECT_EQ(PM_TERMINAL_SUPERSEDED, axis.status.last_terminal_result);
	EXPECT_EQ(3U, axis.status.last_terminal_command_id);
	EXPECT_TRUE(axis.status.command_active);
	EXPECT_EQ(350, axis.status.active_endpoint_ticks);
}

TEST_F(PmMotion, RejectedReplacementLeavesActiveMoveUntouched) {
	prepare_referenced_axis();
	pm_command_t absolute = command(3, PM_COMMAND_MOVE_ABSOLUTE, 300);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &absolute));
	pm_axis_tick(&axis);
	pm_command_t unsupported = command(4, PM_COMMAND_VELOCITY, 20, true);
	EXPECT_EQ(PM_RESULT_UNSUPPORTED, pm_axis_submit(&axis, &unsupported));
	EXPECT_EQ(3U, axis.status.active_command_id);
	EXPECT_EQ(300, axis.status.active_endpoint_ticks);
}

TEST_F(PmMotion, TerminalResultIsRecordedExactlyOnce) {
	prepare_referenced_axis();
	pm_command_t absolute = command(3, PM_COMMAND_MOVE_ABSOLUTE, 300);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &absolute));
	pm_axis_tick(&axis);

	EXPECT_TRUE(pm_axis_finish(&axis, 3, PM_TERMINAL_COMPLETED));
	EXPECT_FALSE(pm_axis_finish(&axis, 3, PM_TERMINAL_COMPLETED));
	EXPECT_EQ(PM_TERMINAL_COMPLETED, axis.status.last_terminal_result);
	EXPECT_EQ(3U, axis.status.last_terminal_command_id);
	EXPECT_EQ(PM_LIFECYCLE_HOLDING, axis.status.lifecycle);
}

TEST_F(PmMotion, RevokeCancelsWorkAndInvalidatesOldGeneration) {
	prepare_referenced_axis();
	pm_command_t absolute = command(3, PM_COMMAND_MOVE_ABSOLUTE, 300);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &absolute));
	pm_axis_tick(&axis);
	pm_axis_revoke_owner(&axis, true);

	EXPECT_FALSE(axis.status.command_active);
	EXPECT_FALSE(axis.status.enabled);
	EXPECT_FALSE(axis.status.referenced);
	EXPECT_EQ(PM_TERMINAL_CANCELLED, axis.status.last_terminal_result);
	EXPECT_EQ(PM_RESULT_STALE_OWNER, pm_axis_submit(&axis, &absolute));
}

TEST_F(PmMotion, RelativeOverflowIsRejectedWithoutSuperseding) {
	prepare_referenced_axis();
	pm_command_t absolute = command(3, PM_COMMAND_MOVE_ABSOLUTE, INT64_MAX);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &absolute));
	pm_axis_tick(&axis);
	pm_command_t relative = command(4, PM_COMMAND_MOVE_RELATIVE, 1, true);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &relative));
	pm_axis_tick(&axis);
	EXPECT_EQ(3U, axis.status.active_command_id);
	EXPECT_EQ(PM_TERMINAL_REJECTED, axis.status.last_terminal_result);
}