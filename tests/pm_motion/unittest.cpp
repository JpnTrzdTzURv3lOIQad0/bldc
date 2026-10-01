#include "gtest/gtest.h"

#include <float.h>
#include <math.h>
#include <string.h>

extern "C" {
#include "motor/pm_interface.h"
#include "motor/pm_feedback.h"
#include "motor/pm_control.h"
#include "motor/pm_trajectory.h"
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

TEST_F(PmMotion, NegativeRelativeOverflowIsRejectedWithoutSuperseding) {
	pm_command_t reference = command(1, PM_COMMAND_SET_REFERENCE, INT64_MIN);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &reference));
	pm_axis_tick(&axis);
	pm_command_t enable = command(2, PM_COMMAND_ENABLE);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &enable));
	pm_axis_tick(&axis);
	pm_command_t relative = command(3, PM_COMMAND_MOVE_RELATIVE, -1);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING,
			pm_axis_submit(&axis, &relative));
	pm_axis_tick(&axis);
	EXPECT_FALSE(axis.status.command_active);
	EXPECT_EQ(PM_TERMINAL_REJECTED, axis.status.last_terminal_result);
	EXPECT_EQ(3U, axis.status.last_terminal_command_id);
}

TEST_F(PmMotion, RelativeMoveRequiresACommandedBaseEndpoint) {
	prepare_referenced_axis();
	axis.status.has_commanded_endpoint = false;
	pm_command_t relative = command(3, PM_COMMAND_MOVE_RELATIVE, 10);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING,
			pm_axis_submit(&axis, &relative));
	pm_axis_tick(&axis);
	EXPECT_FALSE(axis.status.command_active);
	EXPECT_EQ(PM_TERMINAL_REJECTED, axis.status.last_terminal_result);
}

TEST_F(PmMotion, NullAndMalformedOwnershipRequestsAreRejected) {
	uint32_t generation = 99;
	EXPECT_EQ(PM_RESULT_INVALID, pm_axis_claim_owner(NULL, 42, &generation));
	EXPECT_EQ(PM_RESULT_INVALID, pm_axis_claim_owner(&axis, 42, NULL));
	EXPECT_EQ(PM_RESULT_INVALID, pm_axis_claim_owner(&axis, 0, &generation));
	EXPECT_EQ(PM_RESULT_BUSY, pm_axis_claim_owner(&axis, 43, &generation));
	EXPECT_EQ(PM_RESULT_INVALID, pm_axis_submit(NULL, NULL));
	EXPECT_EQ(PM_RESULT_INVALID, pm_axis_submit(&axis, NULL));

	pm_command_t invalid = command(1, PM_COMMAND_ENABLE);
	invalid.command_id = 0;
	EXPECT_EQ(PM_RESULT_INVALID, pm_axis_submit(&axis, &invalid));
	invalid = command(1, PM_COMMAND_ENABLE);
	invalid.type = (pm_command_type_t)-1;
	EXPECT_EQ(PM_RESULT_INVALID, pm_axis_submit(&axis, &invalid));
	invalid = command(1, PM_COMMAND_ENABLE);
	invalid.type = (pm_command_type_t)(PM_COMMAND_CLEAR_FAULT + 1);
	EXPECT_EQ(PM_RESULT_INVALID, pm_axis_submit(&axis, &invalid));
	invalid = command(1, PM_COMMAND_ENABLE);
	invalid.owner_session = 0;
	EXPECT_EQ(PM_RESULT_INVALID, pm_axis_submit(&axis, &invalid));
	invalid = command(1, PM_COMMAND_ENABLE);
	invalid.owner_generation = 0;
	EXPECT_EQ(PM_RESULT_INVALID, pm_axis_submit(&axis, &invalid));
}

TEST_F(PmMotion, OwnerGenerationSkipsZeroOnWrap) {
	pm_axis_revoke_owner(&axis, false);
	axis.status.owner_generation = UINT32_MAX;
	uint32_t generation = 0;
	EXPECT_EQ(PM_RESULT_ACCEPTED_PENDING,
			pm_axis_claim_owner(&axis, 43, &generation));
	EXPECT_EQ(1U, generation);
	EXPECT_EQ(1U, axis.status.owner_generation);
}

TEST_F(PmMotion, ActiveCommandRequiresExplicitSupportedReplacement) {
	prepare_referenced_axis();
	pm_command_t absolute = command(3, PM_COMMAND_MOVE_ABSOLUTE, 300);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &absolute));
	pm_axis_tick(&axis);
	pm_command_t without_replace = command(4, PM_COMMAND_MOVE_ABSOLUTE, 400);
	EXPECT_EQ(PM_RESULT_BUSY, pm_axis_submit(&axis, &without_replace));
	pm_command_t velocity_replace = command(5, PM_COMMAND_VELOCITY, 10, true);
	EXPECT_EQ(PM_RESULT_UNSUPPORTED,
			pm_axis_submit(&axis, &velocity_replace));
	EXPECT_EQ(3U, axis.status.active_command_id);

	pm_command_t valid_replace = command(6, PM_COMMAND_MOVE_ABSOLUTE,
			500, true);
	EXPECT_EQ(PM_RESULT_ACCEPTED_PENDING,
			pm_axis_submit(&axis, &valid_replace));
	EXPECT_EQ(PM_RESULT_BUSY,
			pm_axis_submit(&axis, &velocity_replace));
}

TEST_F(PmMotion, ReplacementWithoutActiveCommandIsInvalid) {
	pm_command_t request = command(1, PM_COMMAND_ENABLE, 0, true);
	EXPECT_EQ(PM_RESULT_INVALID, pm_axis_submit(&axis, &request));
}

TEST_F(PmMotion, EnableAndReferenceTransitionsRejectUnsafeOrder) {
	pm_command_t move = command(1, PM_COMMAND_MOVE_ABSOLUTE, 100);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &move));
	pm_axis_tick(&axis);
	EXPECT_EQ(PM_TERMINAL_REJECTED, axis.status.last_terminal_result);
	EXPECT_FALSE(axis.status.command_active);

	pm_command_t reference = command(2, PM_COMMAND_SET_REFERENCE, 0);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING,
			pm_axis_submit(&axis, &reference));
	pm_axis_tick(&axis);
	pm_command_t enable = command(3, PM_COMMAND_ENABLE);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &enable));
	pm_axis_tick(&axis);
	EXPECT_TRUE(axis.status.enabled);
	EXPECT_TRUE(axis.status.referenced);
	EXPECT_EQ(PM_LIFECYCLE_READY, axis.status.lifecycle);

	reference = command(4, PM_COMMAND_SET_REFERENCE, 5);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING,
			pm_axis_submit(&axis, &reference));
	pm_axis_tick(&axis);
	EXPECT_EQ(PM_TERMINAL_REJECTED, axis.status.last_terminal_result);
	EXPECT_EQ(4U, axis.status.last_terminal_command_id);
}

TEST_F(PmMotion, HomeRunsUnreferencedAndCompletionSetsReference) {
	pm_command_t enable = command(1, PM_COMMAND_ENABLE);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &enable));
	pm_axis_tick(&axis);
	EXPECT_EQ(PM_LIFECYCLE_UNREFERENCED, axis.status.lifecycle);
	pm_command_t home = command(2, PM_COMMAND_HOME);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &home));
	pm_axis_tick(&axis);
	EXPECT_EQ(PM_LIFECYCLE_HOMING, axis.status.lifecycle);
	EXPECT_FALSE(axis.status.active_has_endpoint);
	EXPECT_TRUE(pm_axis_finish(&axis, 2, PM_TERMINAL_COMPLETED));
	EXPECT_TRUE(axis.status.referenced);
	EXPECT_EQ(PM_LIFECYCLE_HOLDING, axis.status.lifecycle);
}

TEST_F(PmMotion, HomeAndVelocityRejectWhenAxisIsNotReadyOrBusy) {
	pm_command_t home = command(1, PM_COMMAND_HOME);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &home));
	pm_axis_tick(&axis);
	EXPECT_EQ(PM_TERMINAL_REJECTED, axis.status.last_terminal_result);

	prepare_referenced_axis();
	pm_command_t velocity = command(3, PM_COMMAND_VELOCITY, 50);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING,
			pm_axis_submit(&axis, &velocity));
	pm_axis_tick(&axis);
	EXPECT_EQ(PM_LIFECYCLE_MOVING, axis.status.lifecycle);
	EXPECT_FALSE(axis.status.active_has_endpoint);
	pm_command_t home_while_moving = command(4, PM_COMMAND_HOME);
	EXPECT_EQ(PM_RESULT_BUSY,
			pm_axis_submit(&axis, &home_while_moving));
	EXPECT_TRUE(pm_axis_finish(&axis, 3, PM_TERMINAL_CANCELLED));
	EXPECT_EQ(PM_LIFECYCLE_READY, axis.status.lifecycle);
}

TEST_F(PmMotion, DeceleratedStopCancelsMoveAndEndsHolding) {
	prepare_referenced_axis();
	pm_command_t move = command(3, PM_COMMAND_MOVE_ABSOLUTE, 300);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &move));
	pm_axis_tick(&axis);
	pm_command_t stop = command(4, PM_COMMAND_STOP_DECELERATED);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &stop));
	pm_axis_tick(&axis);
	EXPECT_EQ(3U, axis.status.last_terminal_command_id);
	EXPECT_EQ(PM_TERMINAL_CANCELLED, axis.status.last_terminal_result);
	EXPECT_EQ(PM_LIFECYCLE_STOPPING, axis.status.lifecycle);
	EXPECT_FALSE(axis.status.active_has_endpoint);
	EXPECT_TRUE(pm_axis_finish(&axis, 4, PM_TERMINAL_COMPLETED));
	EXPECT_EQ(PM_LIFECYCLE_HOLDING, axis.status.lifecycle);
}

TEST_F(PmMotion, StopRejectsWhenDisabled) {
	pm_command_t stop = command(1, PM_COMMAND_STOP_DECELERATED);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &stop));
	pm_axis_tick(&axis);
	EXPECT_EQ(PM_TERMINAL_REJECTED, axis.status.last_terminal_result);
	EXPECT_FALSE(axis.status.command_active);
}

TEST_F(PmMotion, DisableAndAbortTerminateActiveWorkDifferently) {
	prepare_referenced_axis();
	pm_command_t move = command(3, PM_COMMAND_MOVE_ABSOLUTE, 300);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &move));
	pm_axis_tick(&axis);
	pm_command_t disable = command(4, PM_COMMAND_DISABLE);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING,
			pm_axis_submit(&axis, &disable));
	pm_axis_tick(&axis);
	EXPECT_FALSE(axis.status.command_active);
	EXPECT_FALSE(axis.status.enabled);
	EXPECT_EQ(PM_TERMINAL_COMPLETED, axis.status.last_terminal_result);
	EXPECT_EQ(4U, axis.status.last_terminal_command_id);

	pm_command_t enable = command(5, PM_COMMAND_ENABLE);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &enable));
	pm_axis_tick(&axis);
	move = command(6, PM_COMMAND_MOVE_ABSOLUTE, 600);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &move));
	pm_axis_tick(&axis);
	pm_command_t abort = command(7, PM_COMMAND_ABORT_RELEASE);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &abort));
	pm_axis_tick(&axis);
	EXPECT_EQ(PM_TERMINAL_ABORTED, axis.status.last_terminal_result);
	EXPECT_EQ(7U, axis.status.last_terminal_command_id);
	EXPECT_FALSE(axis.status.enabled);
	EXPECT_EQ(PM_LIFECYCLE_DISABLED, axis.status.lifecycle);
}

TEST_F(PmMotion, RevocationCancelsPendingCommandAndAdvancesGeneration) {
	pm_command_t request = command(1, PM_COMMAND_SET_REFERENCE, 12);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &request));
	uint32_t old_generation = axis.status.owner_generation;
	pm_axis_revoke_owner(&axis, false);
	EXPECT_FALSE(axis.status.command_pending);
	EXPECT_EQ(PM_TERMINAL_CANCELLED, axis.status.last_terminal_result);
	EXPECT_EQ(1U, axis.status.last_terminal_command_id);
	EXPECT_NE(old_generation, axis.status.owner_generation);
	uint32_t new_generation = 0;
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING,
			pm_axis_claim_owner(&axis, 43, &new_generation));
	EXPECT_EQ(axis.status.owner_generation, new_generation);
}

TEST_F(PmMotion, ClearFaultRequiresAllAcknowledgementsAndLeavesDisabled) {
	pm_axis_latch_fault(&axis, 17);
	EXPECT_EQ(PM_LIFECYCLE_FAULTED, axis.status.lifecycle);
	EXPECT_EQ(17U, axis.status.fault_code);
	pm_command_t enable = command(1, PM_COMMAND_ENABLE);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &enable));
	pm_axis_tick(&axis);
	EXPECT_EQ(PM_TERMINAL_REJECTED, axis.status.last_terminal_result);
	EXPECT_FALSE(axis.status.enabled);

	const bool clear_inputs[] = {false, true, true, true};
	const bool valid_feedback[] = {true, false, true, true};
	const bool acknowledged[] = {true, true, false, true};
	for (unsigned int index = 0; index < 4; index++) {
		pm_command_t clear = command(index + 2, PM_COMMAND_CLEAR_FAULT);
		clear.fault_inputs_clear = clear_inputs[index];
		clear.feedback_valid = valid_feedback[index];
		clear.acknowledged = acknowledged[index];
		ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING,
				pm_axis_submit(&axis, &clear));
		pm_axis_tick(&axis);
		if (index < 3) {
			EXPECT_EQ(PM_TERMINAL_REJECTED,
					axis.status.last_terminal_result);
			EXPECT_EQ(PM_LIFECYCLE_FAULTED, axis.status.lifecycle);
		} else {
			EXPECT_EQ(PM_TERMINAL_COMPLETED,
					axis.status.last_terminal_result);
			EXPECT_EQ(PM_LIFECYCLE_DISABLED, axis.status.lifecycle);
			EXPECT_EQ(0U, axis.status.fault_code);
			EXPECT_FALSE(axis.status.enabled);
		}
	}
}

TEST_F(PmMotion, FaultCancelsActiveAndPendingCommandsAndBlocksRestart) {
	prepare_referenced_axis();
	pm_command_t move = command(3, PM_COMMAND_MOVE_ABSOLUTE, 300);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &move));
	pm_axis_tick(&axis);
	pm_command_t stop = command(4, PM_COMMAND_STOP_DECELERATED);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &stop));
	pm_axis_latch_fault(&axis, 21);
	EXPECT_FALSE(axis.status.command_active);
	EXPECT_FALSE(axis.status.command_pending);
	EXPECT_FALSE(axis.status.enabled);
	EXPECT_EQ(PM_LIFECYCLE_FAULTED, axis.status.lifecycle);
	EXPECT_EQ(21U, axis.status.fault_code);
	EXPECT_EQ(4U, axis.status.last_terminal_command_id);
	EXPECT_EQ(PM_TERMINAL_REJECTED, axis.status.last_terminal_result);
}

TEST_F(PmMotion, FinishRejectsWrongCommandAndInvalidTerminalValues) {
	prepare_referenced_axis();
	pm_command_t velocity = command(3, PM_COMMAND_VELOCITY, 5);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING,
			pm_axis_submit(&axis, &velocity));
	pm_axis_tick(&axis);
	EXPECT_FALSE(pm_axis_finish(&axis, 4, PM_TERMINAL_COMPLETED));
	EXPECT_FALSE(pm_axis_finish(&axis, 3, PM_TERMINAL_NONE));
	EXPECT_FALSE(pm_axis_finish(&axis, 3,
			(pm_terminal_result_t)(PM_TERMINAL_REJECTED + 1)));
	EXPECT_TRUE(pm_axis_finish(&axis, 3, PM_TERMINAL_ABORTED));
	EXPECT_EQ(PM_LIFECYCLE_READY, axis.status.lifecycle);
	EXPECT_FALSE(pm_axis_finish(&axis, 3, PM_TERMINAL_COMPLETED));
	EXPECT_FALSE(pm_axis_finish(NULL, 3, PM_TERMINAL_COMPLETED));
}

TEST_F(PmMotion, FaultedFinishAndNullStatusCallsAreSafe) {
	EXPECT_FALSE(pm_axis_finish(NULL, 1, PM_TERMINAL_COMPLETED));
	pm_axis_get_status(NULL, NULL);
	pm_axis_status_t status = {};
	pm_axis_get_status(&axis, NULL);
	pm_axis_get_status(&axis, &status);
	EXPECT_EQ(PM_LIFECYCLE_DISABLED, status.lifecycle);

	prepare_referenced_axis();
	pm_command_t home = command(3, PM_COMMAND_HOME);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &home));
	pm_axis_tick(&axis);
	EXPECT_TRUE(pm_axis_finish(&axis, 3, PM_TERMINAL_FAULTED));
	EXPECT_EQ(PM_LIFECYCLE_FAULTED, axis.status.lifecycle);
	EXPECT_FALSE(axis.status.enabled);
}

TEST_F(PmMotion, InvalidCommandFromTickOwnerIsRejected) {
	pm_command_t request = command(1, PM_COMMAND_ENABLE);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_axis_submit(&axis, &request));
	axis.pending_command.owner_generation++;
	pm_axis_tick(&axis);
	EXPECT_FALSE(axis.status.command_pending);
	EXPECT_FALSE(axis.status.enabled);
	EXPECT_EQ(1U, axis.status.last_terminal_command_id);
	EXPECT_EQ(PM_TERMINAL_REJECTED, axis.status.last_terminal_result);
}

class PmTrajectoryTest : public testing::Test {
protected:
	static const float max_dt;

	bool run_to_completion(pm_trajectory_t *trajectory, float dt = 0.01f) {
		for (int step = 0; step < 20000; step++) {
			pm_trajectory_result_t result = pm_trajectory_advance(trajectory, dt);
			if (result == PM_TRAJECTORY_RESULT_COMPLETE) {
				return true;
			}
			if (result != PM_TRAJECTORY_RESULT_RUNNING) {
				return false;
			}
		}
		return false;
	}

	pm_trajectory_result_t start(pm_trajectory_t *trajectory, float position,
			float velocity, float target, float max_velocity = 5.0f,
			float acceleration = 2.0f, float deceleration = 3.0f) {
		return pm_trajectory_start(trajectory, position, velocity, target,
				max_velocity, acceleration, deceleration, max_dt);
	}
};

const float PmTrajectoryTest::max_dt = 0.1f;

TEST_F(PmTrajectoryTest, ZeroDistanceAtRestCompletesImmediately) {
	pm_trajectory_t trajectory = {};
	EXPECT_EQ(PM_TRAJECTORY_RESULT_COMPLETE,
			start(&trajectory, 12.0f, 0.0f, 12.0f));
	EXPECT_EQ(PM_TRAJECTORY_COMPLETE, trajectory.state);
	EXPECT_EQ(0U, trajectory.phase_count);
	EXPECT_FLOAT_EQ(12.0f, trajectory.position);
	EXPECT_FLOAT_EQ(0.0f, trajectory.velocity);
}

TEST_F(PmTrajectoryTest, TriangularProfileRespectsLimitsAndExactEndpoint) {
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 0.0f, 0.0f, 1.0f, 4.0f, 2.0f, 3.0f));
	float previous_velocity = 0.0f;
	float peak_velocity = 0.0f;
	for (int step = 0; step < 200 &&
			trajectory.state == PM_TRAJECTORY_RUNNING; step++) {
		pm_trajectory_result_t result = pm_trajectory_advance(&trajectory,
				0.02f);
		ASSERT_TRUE(result == PM_TRAJECTORY_RESULT_RUNNING ||
				result == PM_TRAJECTORY_RESULT_COMPLETE);
		EXPECT_LE(trajectory.velocity, 4.0f + 0.0001f);
		EXPECT_GE(trajectory.velocity, -0.0001f);
		EXPECT_LE(fabsf(trajectory.velocity - previous_velocity),
				3.0f * 0.02f + 0.0001f);
		EXPECT_GE(trajectory.position, -0.0001f);
		EXPECT_LE(trajectory.position, 1.0001f);
		peak_velocity = fmaxf(peak_velocity, trajectory.velocity);
		previous_velocity = trajectory.velocity;
	}
	EXPECT_EQ(PM_TRAJECTORY_COMPLETE, trajectory.state);
	EXPECT_LT(peak_velocity, 4.0f);
	EXPECT_FLOAT_EQ(1.0f, trajectory.position);
	EXPECT_FLOAT_EQ(0.0f, trajectory.velocity);
}

TEST_F(PmTrajectoryTest, LongMoveIncludesCruiseAtVelocityLimit) {
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 0.0f, 0.0f, 50.0f, 5.0f, 2.0f, 3.0f));
	ASSERT_EQ(3U, trajectory.phase_count);
	EXPECT_FLOAT_EQ(0.0f, trajectory.phases[1].acceleration);
	EXPECT_FLOAT_EQ(5.0f, trajectory.phases[1].end_velocity);
	ASSERT_TRUE(run_to_completion(&trajectory));
	EXPECT_FLOAT_EQ(50.0f, trajectory.position);
	EXPECT_FLOAT_EQ(0.0f, trajectory.velocity);
}

TEST_F(PmTrajectoryTest, NegativeDirectionProfileReachesExactEndpoint) {
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 5.0f, 0.0f, -3.0f, 2.0f, 1.0f, 1.5f));
	ASSERT_TRUE(run_to_completion(&trajectory));
	EXPECT_FLOAT_EQ(-3.0f, trajectory.position);
	EXPECT_FLOAT_EQ(0.0f, trajectory.velocity);
}

TEST_F(PmTrajectoryTest, NonzeroVelocityTowardTargetIsPreservedAtStart) {
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 0.0f, 1.0f, 5.0f));
	EXPECT_FLOAT_EQ(0.0f, trajectory.position);
	EXPECT_FLOAT_EQ(1.0f, trajectory.velocity);
	ASSERT_TRUE(run_to_completion(&trajectory));
	EXPECT_FLOAT_EQ(5.0f, trajectory.position);
	EXPECT_FLOAT_EQ(0.0f, trajectory.velocity);
}

TEST_F(PmTrajectoryTest, InitialVelocityAwayFromTargetIsBrakedFirst) {
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 0.0f, -2.0f, 10.0f, 5.0f, 2.0f, 1.0f));
	ASSERT_GT(trajectory.phase_count, 1U);
	EXPECT_FLOAT_EQ(1.0f, trajectory.phases[0].acceleration);
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			pm_trajectory_advance(&trajectory, 0.1f));
	EXPECT_LT(trajectory.position, 0.0f);
	EXPECT_FLOAT_EQ(-1.9f, trajectory.velocity);
	ASSERT_TRUE(run_to_completion(&trajectory));
	EXPECT_FLOAT_EQ(10.0f, trajectory.position);
}

TEST_F(PmTrajectoryTest, StopsPastTargetThenReversesWithinBounds) {
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 0.0f, 4.0f, 2.0f, 5.0f, 2.0f, 1.0f));
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			pm_trajectory_advance(&trajectory, 0.1f));
	EXPECT_FLOAT_EQ(-1.0f, trajectory.phases[0].acceleration);
	bool moved_past_target = false;
	bool reversed = false;
	for (int step = 0; step < 500 &&
			trajectory.state == PM_TRAJECTORY_RUNNING; step++) {
		pm_trajectory_result_t result = pm_trajectory_advance(&trajectory,
				0.02f);
		ASSERT_TRUE(result == PM_TRAJECTORY_RESULT_RUNNING ||
				result == PM_TRAJECTORY_RESULT_COMPLETE);
		moved_past_target |= trajectory.position > 2.0f;
		reversed |= trajectory.velocity < 0.0f;
	}
	EXPECT_TRUE(moved_past_target);
	EXPECT_TRUE(reversed);
	EXPECT_EQ(PM_TRAJECTORY_COMPLETE, trajectory.state);
	EXPECT_FLOAT_EQ(2.0f, trajectory.position);
	EXPECT_FLOAT_EQ(0.0f, trajectory.velocity);
}

TEST_F(PmTrajectoryTest, SamePositionWithVelocityBrakesAndReturns) {
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 3.0f, 2.0f, 3.0f, 4.0f, 2.0f, 1.0f));
	ASSERT_TRUE(run_to_completion(&trajectory));
	EXPECT_FLOAT_EQ(3.0f, trajectory.position);
	EXPECT_FLOAT_EQ(0.0f, trajectory.velocity);
}

TEST_F(PmTrajectoryTest, OverspeedInitialStateDeceleratesBeforeCruising) {
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 0.0f, 6.0f, 100.0f, 3.0f, 2.0f, 4.0f));
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			pm_trajectory_advance(&trajectory, 0.05f));
	EXPECT_FLOAT_EQ(-4.0f, trajectory.phases[0].acceleration);
	EXPECT_FLOAT_EQ(5.8f, trajectory.velocity);
	ASSERT_TRUE(run_to_completion(&trajectory));
	EXPECT_FLOAT_EQ(100.0f, trajectory.position);
	EXPECT_FLOAT_EQ(0.0f, trajectory.velocity);
}

TEST_F(PmTrajectoryTest, OverspeedReversalUsesBoundedMaximumPhaseCount) {
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 0.0f, 10.0f, 1.0f, 3.0f, 2.0f, 2.0f));
	ASSERT_EQ(PM_TRAJECTORY_MAX_PHASES, trajectory.phase_count);
	EXPECT_FLOAT_EQ(-2.0f, trajectory.phases[0].acceleration);
	EXPECT_FLOAT_EQ(-2.0f, trajectory.phases[1].acceleration);
	EXPECT_FLOAT_EQ(-2.0f, trajectory.phases[2].acceleration);
	EXPECT_FLOAT_EQ(0.0f, trajectory.phases[3].acceleration);
	EXPECT_FLOAT_EQ(2.0f, trajectory.phases[4].acceleration);
	ASSERT_TRUE(run_to_completion(&trajectory, 0.05f));
	EXPECT_FLOAT_EQ(1.0f, trajectory.position);
	EXPECT_FLOAT_EQ(0.0f, trajectory.velocity);
}

TEST_F(PmTrajectoryTest, ExactStoppingDistanceClampsToRequestedEndpoint) {
	const float velocity = 1.1f;
	const float deceleration = 0.7f;
	const float stopping_position = velocity * velocity /
			(2.0f * deceleration);
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 0.0f, velocity, stopping_position, 4.0f,
				2.0f, deceleration));
	ASSERT_EQ(1U, trajectory.phase_count);
	EXPECT_EQ(stopping_position, trajectory.phases[0].end_position);
	ASSERT_TRUE(run_to_completion(&trajectory, 0.05f));
	EXPECT_FLOAT_EQ(stopping_position, trajectory.position);
	EXPECT_FLOAT_EQ(0.0f, trajectory.velocity);
}

TEST_F(PmTrajectoryTest, RepeatedBoundarySizedStepsFinishWithoutStalling) {
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			pm_trajectory_start_stop(&trajectory, 0.0f, 1.0f, 1.0f,
				0.1f));
	for (int step = 0; step < 10 &&
			trajectory.state == PM_TRAJECTORY_RUNNING; step++) {
		pm_trajectory_result_t result = pm_trajectory_advance(&trajectory,
				0.1f);
		ASSERT_TRUE(result == PM_TRAJECTORY_RESULT_RUNNING ||
				result == PM_TRAJECTORY_RESULT_COMPLETE);
	}
	EXPECT_EQ(PM_TRAJECTORY_COMPLETE, trajectory.state);
	EXPECT_FLOAT_EQ(0.5f, trajectory.position);
	EXPECT_FLOAT_EQ(0.0f, trajectory.velocity);
}

TEST_F(PmTrajectoryTest, GeneratedInitialStatesRespectPhaseAndTerminalBounds) {
	const float initial_velocities[] = {-6.0f, -1.0f, 0.0f, 2.0f, 7.0f};
	const float targets[] = {-8.0f, 0.0f, 2.0f, 15.0f};
	for (unsigned int velocity_index = 0;
			velocity_index < sizeof(initial_velocities) /
				sizeof(initial_velocities[0]); velocity_index++) {
		for (unsigned int target_index = 0;
				target_index < sizeof(targets) / sizeof(targets[0]);
				target_index++) {
			pm_trajectory_t trajectory = {};
			float initial_velocity = initial_velocities[velocity_index];
			float target = targets[target_index];
			pm_trajectory_result_t result = start(&trajectory, 0.0f,
					initial_velocity, target, 3.0f, 1.5f, 2.5f);
			ASSERT_TRUE(result == PM_TRAJECTORY_RESULT_RUNNING ||
					result == PM_TRAJECTORY_RESULT_COMPLETE);
			ASSERT_LE(trajectory.phase_count, PM_TRAJECTORY_MAX_PHASES);
			for (unsigned int phase_index = 0;
					phase_index < trajectory.phase_count; phase_index++) {
				float acceleration =
						trajectory.phases[phase_index].acceleration;
				float start_velocity = phase_index == 0 ? initial_velocity :
						trajectory.phases[phase_index - 1].end_velocity;
				bool braking = start_velocity != 0.0f &&
						acceleration * start_velocity < 0.0f;
				EXPECT_LE(fabsf(acceleration), braking ? 2.5f : 1.5f);
			}
			ASSERT_TRUE(run_to_completion(&trajectory, 0.037f));
			EXPECT_FLOAT_EQ(target, trajectory.position);
			EXPECT_FLOAT_EQ(0.0f, trajectory.velocity);
		}
	}
}

TEST_F(PmTrajectoryTest, RetargetStartsFromCommandedPositionAndVelocity) {
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 0.0f, 0.0f, 100.0f, 10.0f, 4.0f, 4.0f));
	for (int step = 0; step < 10; step++) {
		ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
				pm_trajectory_advance(&trajectory, 0.1f));
	}
	float position_before_retarget = trajectory.position;
	float velocity_before_retarget = trajectory.velocity;
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			pm_trajectory_retarget(&trajectory, 0.5f, 2.0f, 1.0f,
				3.0f, max_dt));
	EXPECT_FLOAT_EQ(position_before_retarget, trajectory.position);
	EXPECT_FLOAT_EQ(velocity_before_retarget, trajectory.velocity);
	EXPECT_GT(trajectory.velocity, 2.0f);
	ASSERT_TRUE(run_to_completion(&trajectory));
	EXPECT_FLOAT_EQ(0.5f, trajectory.position);
	EXPECT_FLOAT_EQ(0.0f, trajectory.velocity);
}

TEST_F(PmTrajectoryTest, ExplicitStopRampsVelocityToZero) {
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			pm_trajectory_start_stop(&trajectory, 10.0f, -3.0f, 2.0f,
				max_dt));
	EXPECT_FLOAT_EQ(7.75f, trajectory.target_position);
	ASSERT_TRUE(run_to_completion(&trajectory, 0.05f));
	EXPECT_FLOAT_EQ(7.75f, trajectory.position);
	EXPECT_FLOAT_EQ(0.0f, trajectory.velocity);
}

TEST_F(PmTrajectoryTest, ZeroVelocityStopCompletesImmediately) {
	pm_trajectory_t trajectory = {};
	EXPECT_EQ(PM_TRAJECTORY_RESULT_COMPLETE,
			pm_trajectory_start_stop(&trajectory, -4.0f, 0.0f, 1.0f,
				max_dt));
	EXPECT_EQ(PM_TRAJECTORY_COMPLETE, trajectory.state);
	EXPECT_FLOAT_EQ(-4.0f, trajectory.position);
}

TEST_F(PmTrajectoryTest, JitteredDtMayCrossMultipleBoundedPhases) {
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 0.0f, 0.0f, 0.25f, 3.0f, 2.0f, 2.0f));
	const float steps[] = {0.03f, 0.1f, 0.07f, 0.04f, 0.09f};
	int index = 0;
	for (int step = 0; step < 100 &&
			trajectory.state == PM_TRAJECTORY_RUNNING; step++) {
		pm_trajectory_result_t result = pm_trajectory_advance(&trajectory,
				steps[index++ % 5]);
		ASSERT_TRUE(result == PM_TRAJECTORY_RESULT_RUNNING ||
				result == PM_TRAJECTORY_RESULT_COMPLETE);
	}
	EXPECT_EQ(PM_TRAJECTORY_COMPLETE, trajectory.state);
	EXPECT_FLOAT_EQ(0.25f, trajectory.position);
	EXPECT_FLOAT_EQ(0.0f, trajectory.velocity);
}

TEST_F(PmTrajectoryTest, InvalidDtDoesNotMutateTrajectory) {
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 0.0f, 0.0f, 10.0f));
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			pm_trajectory_advance(&trajectory, 0.05f));
	const float invalid_steps[] = {0.0f, -0.01f, 0.1001f, NAN, INFINITY};
	for (unsigned int index = 0;
			index < sizeof(invalid_steps) / sizeof(invalid_steps[0]); index++) {
		pm_trajectory_t before = trajectory;
		EXPECT_EQ(PM_TRAJECTORY_RESULT_INVALID_DT,
				pm_trajectory_advance(&trajectory, invalid_steps[index]));
		EXPECT_EQ(before.state, trajectory.state);
		EXPECT_FLOAT_EQ(before.position, trajectory.position);
		EXPECT_FLOAT_EQ(before.velocity, trajectory.velocity);
		EXPECT_EQ(before.phase_index, trajectory.phase_index);
		EXPECT_FLOAT_EQ(before.phase_elapsed, trajectory.phase_elapsed);
	}
}

TEST_F(PmTrajectoryTest, InvalidInputsAndNumericOverflowAreRejected) {
	pm_trajectory_t trajectory = {};
	EXPECT_EQ(PM_TRAJECTORY_RESULT_INVALID_ARGUMENT,
			pm_trajectory_start(NULL, 0.0f, 0.0f, 1.0f,
				1.0f, 1.0f, 1.0f, max_dt));
	EXPECT_EQ(PM_TRAJECTORY_RESULT_INVALID_ARGUMENT,
			start(&trajectory, NAN, 0.0f, 1.0f));
	EXPECT_EQ(PM_TRAJECTORY_RESULT_INVALID_ARGUMENT,
			start(&trajectory, 0.0f, INFINITY, 1.0f));
	EXPECT_EQ(PM_TRAJECTORY_RESULT_INVALID_ARGUMENT,
			start(&trajectory, 0.0f, 0.0f, 1.0f, 0.0f));
	EXPECT_EQ(PM_TRAJECTORY_RESULT_INVALID_ARGUMENT,
			start(&trajectory, 0.0f, 0.0f, 1.0f, 1.0f, -1.0f));
	EXPECT_EQ(PM_TRAJECTORY_RESULT_INVALID_ARGUMENT,
			pm_trajectory_start_stop(&trajectory, 0.0f, 1.0f, 0.0f,
				max_dt));
	EXPECT_EQ(PM_TRAJECTORY_RESULT_NUMERIC_ERROR,
			start(&trajectory, -FLT_MAX, 0.0f, FLT_MAX));
}

TEST_F(PmTrajectoryTest, InvalidRetargetLeavesActiveProfileUnchanged) {
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 0.0f, 0.0f, 10.0f));
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			pm_trajectory_advance(&trajectory, 0.05f));
	pm_trajectory_t before = trajectory;
	EXPECT_EQ(PM_TRAJECTORY_RESULT_INVALID_ARGUMENT,
			pm_trajectory_retarget(&trajectory, 3.0f, 0.0f, 1.0f,
				1.0f, max_dt));
	EXPECT_FLOAT_EQ(before.position, trajectory.position);
	EXPECT_FLOAT_EQ(before.velocity, trajectory.velocity);
	EXPECT_FLOAT_EQ(before.target_position, trajectory.target_position);
	EXPECT_EQ(before.phase_index, trajectory.phase_index);
	EXPECT_FLOAT_EQ(before.phase_elapsed, trajectory.phase_elapsed);
}

TEST_F(PmTrajectoryTest, NullInactiveAndCorruptedRuntimeStatesAreRejected) {
	pm_trajectory_t trajectory = {};
	EXPECT_EQ(PM_TRAJECTORY_RESULT_INVALID_ARGUMENT,
			pm_trajectory_retarget(NULL, 1.0f, 1.0f, 1.0f, 1.0f,
				max_dt));
	EXPECT_EQ(PM_TRAJECTORY_RESULT_INVALID_ARGUMENT,
			pm_trajectory_retarget(&trajectory, 1.0f, 1.0f, 1.0f,
				1.0f, max_dt));
	EXPECT_EQ(PM_TRAJECTORY_RESULT_INVALID_ARGUMENT,
			pm_trajectory_advance(NULL, 0.01f));
	EXPECT_EQ(PM_TRAJECTORY_RESULT_INVALID_ARGUMENT,
			pm_trajectory_start_stop(NULL, 0.0f, 1.0f, 1.0f, max_dt));

	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 0.0f, 0.0f, 10.0f));
	trajectory.phase_index = trajectory.phase_count;
	EXPECT_EQ(PM_TRAJECTORY_RESULT_INVALID_ARGUMENT,
			pm_trajectory_advance(&trajectory, 0.01f));

	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 0.0f, 0.0f, 10.0f));
	trajectory.phases[trajectory.phase_index].duration =
			trajectory.phase_elapsed;
	EXPECT_EQ(PM_TRAJECTORY_RESULT_NUMERIC_ERROR,
			pm_trajectory_advance(&trajectory, 0.01f));
}

TEST_F(PmTrajectoryTest, FiniteInputsThatOverflowProfileArithmeticAreRejected) {
	pm_trajectory_t trajectory = {};
	EXPECT_EQ(PM_TRAJECTORY_RESULT_NUMERIC_ERROR,
			start(&trajectory, FLT_MAX / 2.0f, FLT_MAX, FLT_MAX / 2.0f,
				1.0f, 1.0f, 1.0f));
	EXPECT_EQ(PM_TRAJECTORY_RESULT_INVALID_ARGUMENT,
			pm_trajectory_start_stop(&trajectory, 0.0f, 1.0f, 1.0f,
				0.0f));
	EXPECT_EQ(PM_TRAJECTORY_RESULT_NUMERIC_ERROR,
			pm_trajectory_start_stop(&trajectory, 0.0f, FLT_MAX,
				FLT_MIN, max_dt));

	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 0.0f, 0.0f, 10.0f));
	trajectory.position = FLT_MAX;
	trajectory.velocity = FLT_MAX;
	trajectory.phases[trajectory.phase_index].acceleration = FLT_MAX;
	EXPECT_EQ(PM_TRAJECTORY_RESULT_NUMERIC_ERROR,
			pm_trajectory_advance(&trajectory, max_dt));
}

TEST_F(PmTrajectoryTest, CompletedProfileRemainsAtExactTerminalState) {
	pm_trajectory_t trajectory = {};
	ASSERT_EQ(PM_TRAJECTORY_RESULT_RUNNING,
			start(&trajectory, 0.0f, 0.0f, 1.0f));
	ASSERT_TRUE(run_to_completion(&trajectory));
	EXPECT_EQ(PM_TRAJECTORY_RESULT_COMPLETE,
			pm_trajectory_advance(&trajectory, 0.05f));
	EXPECT_FLOAT_EQ(1.0f, trajectory.position);
	EXPECT_FLOAT_EQ(0.0f, trajectory.velocity);
}

class PmFeedbackTest : public testing::Test {
protected:
	pm_feedback_t feedback;
	pm_feedback_config_t config;

	void SetUp() override {
		config = {};
		config.axis_id = 2;
		config.source_id = 4;
		config.counts_per_revolution = 360;
		config.max_counts_per_second = 1000;
		config.max_sample_gap_ms = 200;
		config.max_sample_age_ms = 100;
		ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
				pm_feedback_init(&feedback, &config));
	}

	pm_feedback_sample_t sample(uint32_t sequence, uint32_t time_ms,
			uint32_t position, uint32_t reset_epoch = 7,
			bool sensor_healthy = true, bool commutation_valid = true) {
		pm_feedback_sample_t result = {};
		result.axis_id = config.axis_id;
		result.source_id = config.source_id;
		result.sequence = sequence;
		result.acquisition_time_ms = time_ms;
		result.reset_epoch = reset_epoch;
		result.position_counts = position;
		result.sensor_healthy = sensor_healthy;
		result.commutation_valid = commutation_valid;
		return result;
	}

	void seed(uint32_t sequence = 1, uint32_t time_ms = 10,
			uint32_t position = 0) {
		pm_feedback_sample_t initial_sample = sample(sequence, time_ms, position);
		ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
				pm_feedback_update(&feedback, &initial_sample, time_ms));
	}
};

TEST_F(PmFeedbackTest, StartsUnreferencedAndAcceptsFreshBaseline) {
	seed(5, 20, 359);
	pm_feedback_status_t status = {};
	EXPECT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_get_status(&feedback, 20, &status));
	EXPECT_TRUE(status.initialized);
	EXPECT_TRUE(status.valid);
	EXPECT_TRUE(status.commutation_valid);
	EXPECT_FALSE(status.referenced);
	EXPECT_EQ(359, status.raw_position_counts);
	EXPECT_EQ(0.0f, status.velocity_counts_per_second);
}

TEST_F(PmFeedbackTest, StatusAndFreshnessReportUninitializedAndExpiredStates) {
	pm_feedback_status_t status = {};
	EXPECT_EQ(PM_FEEDBACK_RESULT_NOT_READY,
			pm_feedback_check_freshness(&feedback, 0));
	EXPECT_EQ(PM_FEEDBACK_RESULT_NOT_READY,
			pm_feedback_get_status(&feedback, 0, &status));
	EXPECT_FALSE(status.initialized);

	pm_feedback_sample_t stale_baseline = sample(1, 0, 10);
	EXPECT_EQ(PM_FEEDBACK_RESULT_STALE,
			pm_feedback_update(&feedback, &stale_baseline, 101));
	EXPECT_FALSE(feedback.initialized);
	EXPECT_FALSE(feedback.valid);
	EXPECT_EQ(PM_FEEDBACK_RESULT_NOT_READY,
			pm_feedback_check_freshness(&feedback, 101));
}

TEST_F(PmFeedbackTest, ZeroElapsedSampleInvalidatesTracking) {
	seed(1, 10, 20);
	pm_feedback_sample_t same_time = sample(2, 10, 21);
	EXPECT_EQ(PM_FEEDBACK_RESULT_STALE,
			pm_feedback_update(&feedback, &same_time, 10));
	EXPECT_FALSE(feedback.valid);
	EXPECT_FALSE(feedback.referenced);
}

TEST_F(PmFeedbackTest, UnwrapsBothDirectionsAcrossModuloBoundary) {
	seed(1, 10, 359);
	pm_feedback_sample_t next_sample = sample(2, 11, 0);
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_update(&feedback, &next_sample, 11));
	EXPECT_EQ(360, feedback.raw_position_counts);
	EXPECT_EQ(1000.0f, feedback.velocity_counts_per_second);

	pm_feedback_init(&feedback, &config);
	seed(1, 10, 0);
	next_sample = sample(2, 11, 359);
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_update(&feedback, &next_sample, 11));
	EXPECT_EQ(-1, feedback.raw_position_counts);
	EXPECT_EQ(-1000.0f, feedback.velocity_counts_per_second);
}

TEST_F(PmFeedbackTest, TracksManyTurnsWithoutRoundingAwayCounts) {
	seed(1, 0, 0);
	const uint32_t positions[] = {100, 200, 300, 40, 140, 240, 340};
	for (unsigned int index = 0;
			index < sizeof(positions) / sizeof(positions[0]); index++) {
		pm_feedback_sample_t next_sample = sample(index + 2,
				(index + 1) * 100, positions[index]);
		ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
				pm_feedback_update(&feedback, &next_sample,
						next_sample.acquisition_time_ms));
	}
	EXPECT_EQ(700, feedback.raw_position_counts);
}

TEST_F(PmFeedbackTest, InversionIsAppliedOnceToUnwrappedMotion) {
	config.inverted = true;
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_init(&feedback, &config));
	seed(1, 10, 359);
	pm_feedback_sample_t next_sample = sample(2, 11, 0);
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_update(&feedback, &next_sample, 11));
	EXPECT_EQ(-360, feedback.raw_position_counts);
	EXPECT_EQ(-1000.0f, feedback.velocity_counts_per_second);
}

TEST_F(PmFeedbackTest, HalfTurnStepIsAmbiguousAndInvalidatesReference) {
	seed();
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_set_reference(&feedback, 0, 10));
	pm_feedback_sample_t ambiguous_sample = sample(2, 20, 180);
	EXPECT_EQ(PM_FEEDBACK_RESULT_AMBIGUOUS,
			pm_feedback_update(&feedback, &ambiguous_sample, 20));
	EXPECT_FALSE(feedback.valid);
	EXPECT_FALSE(feedback.referenced);
}

TEST_F(PmFeedbackTest, PhysicallyImpossibleJumpInvalidatesTracking) {
	config.max_counts_per_second = 100;
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_init(&feedback, &config));
	seed();
	pm_feedback_sample_t jump_sample = sample(2, 110, 20);
	EXPECT_EQ(PM_FEEDBACK_RESULT_IMPOSSIBLE_MOTION,
			pm_feedback_update(&feedback, &jump_sample, 110));
	EXPECT_FALSE(feedback.valid);
	EXPECT_FALSE(feedback.referenced);
}

TEST_F(PmFeedbackTest, StaleSamplesAndLongGapsInvalidateReference) {
	seed();
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_set_reference(&feedback, 0, 10));
	EXPECT_EQ(PM_FEEDBACK_RESULT_STALE,
			pm_feedback_check_freshness(&feedback, 111));
	EXPECT_FALSE(feedback.valid);
	EXPECT_FALSE(feedback.referenced);

	seed(1, 10, 20);
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_set_reference(&feedback, 5, 10));
	pm_feedback_sample_t long_gap_sample = sample(2, 211, 21);
	EXPECT_EQ(PM_FEEDBACK_RESULT_STALE,
			pm_feedback_update(&feedback, &long_gap_sample, 211));
	EXPECT_FALSE(feedback.valid);
	EXPECT_FALSE(feedback.referenced);
}

TEST_F(PmFeedbackTest, ReferenceCannotBeSetFromExpiredSample) {
	seed(1, 10, 20);
	EXPECT_EQ(PM_FEEDBACK_RESULT_STALE,
			pm_feedback_set_reference(&feedback, 0, 111));
	EXPECT_FALSE(feedback.valid);
	EXPECT_FALSE(feedback.referenced);
}

TEST_F(PmFeedbackTest, DuplicateOrOldSequenceDoesNotRefreshOrMovePosition) {
	seed(10, 10, 100);
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_set_reference(&feedback, 0, 10));
	pm_feedback_sample_t duplicate = sample(10, 20, 101);
	EXPECT_EQ(PM_FEEDBACK_RESULT_STALE,
			pm_feedback_update(&feedback, &duplicate, 20));
	pm_feedback_sample_t old_sample = sample(9, 20, 101);
	EXPECT_EQ(PM_FEEDBACK_RESULT_STALE,
			pm_feedback_update(&feedback, &old_sample, 20));
	EXPECT_TRUE(feedback.valid);
	EXPECT_TRUE(feedback.referenced);
	EXPECT_EQ(100, feedback.raw_position_counts);
	EXPECT_EQ(10U, feedback.last_sequence);
}

TEST_F(PmFeedbackTest, SequenceAndTimestampCountersMayWrap) {
	seed(UINT32_MAX, UINT32_MAX - 10U, 359);
	pm_feedback_sample_t wrapped_sample = sample(0, 5, 0);
	EXPECT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_update(&feedback, &wrapped_sample, 5));
	EXPECT_EQ(360, feedback.raw_position_counts);
}

TEST_F(PmFeedbackTest, WrongAxisOrSourceCannotMutateFeedback) {
	seed();
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_set_reference(&feedback, 0, 10));
	pm_feedback_t before = feedback;
	pm_feedback_sample_t wrong_source = sample(2, 20, 1);
	wrong_source.source_id++;
	EXPECT_EQ(PM_FEEDBACK_RESULT_IDENTITY_MISMATCH,
			pm_feedback_update(&feedback, &wrong_source, 20));
	EXPECT_EQ(before.raw_position_counts, feedback.raw_position_counts);
	EXPECT_EQ(before.last_sequence, feedback.last_sequence);
	EXPECT_TRUE(feedback.valid);
	EXPECT_TRUE(feedback.referenced);
}

TEST_F(PmFeedbackTest, BadSensorHealthOrOutOfRangeSampleInvalidates) {
	seed();
	pm_feedback_sample_t unhealthy = sample(2, 20, 1, 7, false);
	EXPECT_EQ(PM_FEEDBACK_RESULT_SENSOR_FAULT,
			pm_feedback_update(&feedback, &unhealthy, 20));
	EXPECT_FALSE(feedback.valid);

	seed();
	pm_feedback_sample_t out_of_range = sample(2, 20, 360);
	EXPECT_EQ(PM_FEEDBACK_RESULT_SENSOR_FAULT,
			pm_feedback_update(&feedback, &out_of_range, 20));
	EXPECT_FALSE(feedback.valid);
}

TEST_F(PmFeedbackTest, ResetEpochRebasesAndDropsReference) {
	seed(10, 10, 300);
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_set_reference(&feedback, 42, 10));
	pm_feedback_sample_t reset_sample = sample(1, 20, 3, 8);
	EXPECT_EQ(PM_FEEDBACK_RESULT_RESET,
			pm_feedback_update(&feedback, &reset_sample, 20));
	EXPECT_TRUE(feedback.valid);
	EXPECT_FALSE(feedback.referenced);
	EXPECT_EQ(3, feedback.raw_position_counts);
	EXPECT_EQ(8U, feedback.reset_epoch);
}

TEST_F(PmFeedbackTest, ReferenceRequiresCommutationAndTracksOffset) {
	seed(1, 10, 100);
	pm_feedback_sample_t uncalibrated = sample(2, 20, 101, 7, true, false);
	ASSERT_EQ(PM_FEEDBACK_RESULT_COMMUTATION_INVALID,
			pm_feedback_update(&feedback, &uncalibrated, 20));
	EXPECT_EQ(PM_FEEDBACK_RESULT_NOT_READY,
			pm_feedback_set_reference(&feedback, 12, 20));
	pm_feedback_sample_t calibrated = sample(3, 30, 102);
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_update(&feedback, &calibrated, 30));
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_set_reference(&feedback, 12, 30));
	pm_feedback_status_t status = {};
	EXPECT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_get_status(&feedback, 30, &status));
	EXPECT_TRUE(status.referenced);
	EXPECT_EQ(12, status.axis_position_counts);
	pm_feedback_sample_t moved = sample(4, 31, 103);
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_update(&feedback, &moved, 31));
	EXPECT_EQ(13, feedback.raw_position_counts -
			feedback.reference_offset_counts);
}

TEST_F(PmFeedbackTest, LosingCommutationValidityClearsReference) {
	seed(1, 10, 100);
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK,
			pm_feedback_set_reference(&feedback, 0, 10));
	pm_feedback_sample_t uncalibrated = sample(2, 20, 101, 7, true, false);
	EXPECT_EQ(PM_FEEDBACK_RESULT_COMMUTATION_INVALID,
			pm_feedback_update(&feedback, &uncalibrated, 20));
	EXPECT_TRUE(feedback.valid);
	EXPECT_FALSE(feedback.commutation_valid);
	EXPECT_FALSE(feedback.referenced);
}

TEST_F(PmFeedbackTest, ReferenceOffsetOverflowIsRejected) {
	seed(1, 10, 1);
	EXPECT_EQ(PM_FEEDBACK_RESULT_OVERFLOW,
			pm_feedback_set_reference(&feedback, INT64_MIN, 10));
	EXPECT_FALSE(feedback.referenced);
}

TEST_F(PmFeedbackTest, UnwrappedCountOverflowInvalidatesTracking) {
	seed();
	feedback.raw_position_counts = INT64_MAX;
	feedback.last_raw_counts = 0;
	pm_feedback_sample_t overflow_sample = sample(2, 11, 1);
	EXPECT_EQ(PM_FEEDBACK_RESULT_OVERFLOW,
			pm_feedback_update(&feedback, &overflow_sample, 11));
	EXPECT_FALSE(feedback.valid);
	EXPECT_FALSE(feedback.referenced);
}

TEST_F(PmFeedbackTest, InvalidConfigurationsAndNullArgumentsAreRejected) {
	EXPECT_EQ(PM_FEEDBACK_RESULT_INVALID_ARGUMENT,
			pm_feedback_init(NULL, &config));
	EXPECT_EQ(PM_FEEDBACK_RESULT_INVALID_ARGUMENT,
			pm_feedback_init(&feedback, NULL));
	pm_feedback_config_t invalid = config;
	invalid.counts_per_revolution = 1;
	EXPECT_EQ(PM_FEEDBACK_RESULT_INVALID_ARGUMENT,
			pm_feedback_init(&feedback, &invalid));
	invalid = config;
	invalid.max_sample_age_ms = 0x80000000U;
	EXPECT_EQ(PM_FEEDBACK_RESULT_INVALID_ARGUMENT,
			pm_feedback_init(&feedback, &invalid));
	invalid = config;
	invalid.max_sample_gap_ms = 0;
	EXPECT_EQ(PM_FEEDBACK_RESULT_INVALID_ARGUMENT,
			pm_feedback_init(&feedback, &invalid));
	invalid = config;
	invalid.max_counts_per_second = 0;
	EXPECT_EQ(PM_FEEDBACK_RESULT_INVALID_ARGUMENT,
			pm_feedback_init(&feedback, &invalid));
	invalid = config;
	invalid.max_sample_age_ms = 0;
	EXPECT_EQ(PM_FEEDBACK_RESULT_INVALID_ARGUMENT,
			pm_feedback_init(&feedback, &invalid));
	EXPECT_EQ(PM_FEEDBACK_RESULT_INVALID_ARGUMENT,
			pm_feedback_update(NULL, NULL, 0));
	EXPECT_EQ(PM_FEEDBACK_RESULT_INVALID_ARGUMENT,
			pm_feedback_check_freshness(NULL, 0));
	EXPECT_EQ(PM_FEEDBACK_RESULT_INVALID_ARGUMENT,
			pm_feedback_set_reference(NULL, 0, 0));
	EXPECT_EQ(PM_FEEDBACK_RESULT_INVALID_ARGUMENT,
			pm_feedback_get_status(&feedback, 0, NULL));
}

class PmControlTest : public testing::Test {
protected:
	pm_control_t control;
	pm_control_config_t config;
	pm_control_input_t input;

	void SetUp() override {
		config = {};
		config.max_velocity_counts_per_second = 100.0f;
		config.position_gain_per_second = 0.1f;
		config.velocity_kp_amps_per_count_per_second = 0.2f;
		config.velocity_ki_amps_per_count = 0.0f;
		config.current_limit_amps = 10.0f;
		config.velocity_filter_time_constant_seconds = 0.0f;
		config.max_dt_seconds = 0.1f;
		config.following_error_limit_counts = 1000;
		ASSERT_EQ(PM_CONTROL_RESULT_OK, pm_control_init(&control, &config));
		input = {};
		input.enabled = true;
		input.feedback_valid = true;
		input.referenced = true;
	}

	pm_control_result_t step(pm_control_output_t *output, float dt = 0.01f) {
		return pm_control_step(&control, &input, dt, output);
	}
};

TEST_F(PmControlTest, PositionErrorUsesSignedUnwrappedDifference) {
	pm_control_output_t output = {};
	input.commanded_position_counts = 20000;
	input.measured_position_counts = -20000;
	ASSERT_EQ(PM_CONTROL_RESULT_OK, step(&output));
	EXPECT_EQ(40000, output.position_error_counts);
	EXPECT_FLOAT_EQ(100.0f,
			output.commanded_velocity_counts_per_second);
	EXPECT_TRUE(output.velocity_saturated);
	EXPECT_TRUE(output.output_valid);

	input.commanded_position_counts = -20000;
	input.measured_position_counts = 20000;
	ASSERT_EQ(PM_CONTROL_RESULT_OK, step(&output));
	EXPECT_EQ(-40000, output.position_error_counts);
	EXPECT_FLOAT_EQ(-100.0f,
			output.commanded_velocity_counts_per_second);
}

TEST_F(PmControlTest, LargePositionErrorSaturatesWithoutIntermediateOverflow) {
	pm_control_output_t output = {};
	config.position_gain_per_second = FLT_MAX;
	ASSERT_EQ(PM_CONTROL_RESULT_OK, pm_control_init(&control, &config));
	input.commanded_position_counts = INT64_MAX;
	input.measured_position_counts = 0;
	ASSERT_EQ(PM_CONTROL_RESULT_OK, step(&output));
	EXPECT_TRUE(output.velocity_saturated);
	EXPECT_FLOAT_EQ(100.0f,
			output.commanded_velocity_counts_per_second);
	EXPECT_TRUE(output.output_valid);
}

TEST_F(PmControlTest, FeedForwardAndPositionCorrectionCombine) {
	pm_control_output_t output = {};
	input.commanded_position_counts = 50;
	input.commanded_velocity_counts_per_second = 3.0f;
	ASSERT_EQ(PM_CONTROL_RESULT_OK, step(&output));
	EXPECT_FLOAT_EQ(8.0f,
			output.commanded_velocity_counts_per_second);
	EXPECT_FLOAT_EQ(1.6f, output.iq_demand_amps);
	EXPECT_FALSE(output.velocity_saturated);
}

TEST_F(PmControlTest, CurrentDemandIsClampedToConfiguredLimit) {
	pm_control_output_t output = {};
	input.commanded_velocity_counts_per_second = 100.0f;
	config.velocity_kp_amps_per_count_per_second = 2.0f;
	ASSERT_EQ(PM_CONTROL_RESULT_OK, pm_control_init(&control, &config));
	ASSERT_EQ(PM_CONTROL_RESULT_OK, step(&output));
	EXPECT_FLOAT_EQ(10.0f, output.iq_demand_amps);
	EXPECT_TRUE(output.current_saturated);
}

TEST_F(PmControlTest, IntegratorAccumulatesWhenUnsaturated) {
	pm_control_output_t output = {};
	config.velocity_kp_amps_per_count_per_second = 0.0f;
	config.velocity_ki_amps_per_count = 0.5f;
	ASSERT_EQ(PM_CONTROL_RESULT_OK, pm_control_init(&control, &config));
	input.commanded_velocity_counts_per_second = 5.0f;
	ASSERT_EQ(PM_CONTROL_RESULT_OK, step(&output, 0.1f));
	EXPECT_FLOAT_EQ(0.0f, output.iq_demand_amps);
	EXPECT_FLOAT_EQ(0.25f, control.velocity_integral_amps);
	ASSERT_EQ(PM_CONTROL_RESULT_OK, step(&output, 0.1f));
	EXPECT_FLOAT_EQ(0.25f, output.iq_demand_amps);
}

TEST_F(PmControlTest, AntiWindupBlocksIntegrationIntoCurrentLimit) {
	pm_control_output_t output = {};
	config.velocity_kp_amps_per_count_per_second = 2.0f;
	config.velocity_ki_amps_per_count = 1.0f;
	config.current_limit_amps = 3.0f;
	ASSERT_EQ(PM_CONTROL_RESULT_OK, pm_control_init(&control, &config));
	input.commanded_velocity_counts_per_second = 100.0f;
	for (int step_index = 0; step_index < 20; step_index++) {
		ASSERT_EQ(PM_CONTROL_RESULT_OK, step(&output, 0.1f));
		EXPECT_FLOAT_EQ(3.0f, output.iq_demand_amps);
		EXPECT_TRUE(output.current_saturated);
		EXPECT_FLOAT_EQ(0.0f, control.velocity_integral_amps);
	}
}

TEST_F(PmControlTest, AntiWindupAllowsIntegratorToUnwindSaturation) {
	pm_control_output_t output = {};
	config.velocity_kp_amps_per_count_per_second = 0.1f;
	config.velocity_ki_amps_per_count = 1.0f;
	config.current_limit_amps = 3.0f;
	ASSERT_EQ(PM_CONTROL_RESULT_OK, pm_control_init(&control, &config));
	control.initialized = true;
	control.velocity_integral_amps = 3.5f;
	input.measured_velocity_counts_per_second = 1.0f;
	ASSERT_EQ(PM_CONTROL_RESULT_OK, step(&output, 0.1f));
	EXPECT_TRUE(output.current_saturated);
	EXPECT_FLOAT_EQ(3.0f, output.iq_demand_amps);
	EXPECT_FLOAT_EQ(3.0f, control.velocity_integral_amps);
}

TEST_F(PmControlTest, VelocityFilterUsesConfiguredTimeConstant) {
	pm_control_output_t output = {};
	config.velocity_filter_time_constant_seconds = 0.2f;
	ASSERT_EQ(PM_CONTROL_RESULT_OK, pm_control_init(&control, &config));
	ASSERT_EQ(PM_CONTROL_RESULT_OK, step(&output, 0.1f));
	input.measured_velocity_counts_per_second = 9.0f;
	ASSERT_EQ(PM_CONTROL_RESULT_OK, step(&output, 0.1f));
	EXPECT_NEAR(3.0f, output.filtered_measured_velocity_counts_per_second,
			0.0001f);
	ASSERT_EQ(PM_CONTROL_RESULT_OK, step(&output, 0.1f));
	EXPECT_NEAR(5.0f, output.filtered_measured_velocity_counts_per_second,
			0.0001f);
}

TEST_F(PmControlTest, FollowingErrorThresholdIsStrictlyExceeded) {
	pm_control_output_t output = {};
	config.position_gain_per_second = 0.0f;
	config.following_error_limit_counts = 10;
	ASSERT_EQ(PM_CONTROL_RESULT_OK, pm_control_init(&control, &config));
	input.commanded_position_counts = 10;
	ASSERT_EQ(PM_CONTROL_RESULT_OK, step(&output));
	EXPECT_FALSE(output.following_error_exceeded);
	input.commanded_position_counts = 11;
	ASSERT_EQ(PM_CONTROL_RESULT_OK, step(&output));
	EXPECT_TRUE(output.following_error_exceeded);
}

TEST_F(PmControlTest, NotReadyInputsProduceZeroAndResetControllerState) {
	pm_control_output_t output = {};
	ASSERT_EQ(PM_CONTROL_RESULT_OK, pm_control_reset(&control, 5.0f));
	control.velocity_integral_amps = 2.0f;
	input.enabled = false;
	EXPECT_EQ(PM_CONTROL_RESULT_NOT_READY, step(&output));
	EXPECT_FALSE(output.output_valid);
	EXPECT_FLOAT_EQ(0.0f, output.iq_demand_amps);
	EXPECT_FALSE(control.initialized);
	EXPECT_FLOAT_EQ(0.0f, control.velocity_integral_amps);

	input.enabled = true;
	input.feedback_valid = false;
	EXPECT_EQ(PM_CONTROL_RESULT_NOT_READY, step(&output));
	input.feedback_valid = true;
	input.referenced = false;
	EXPECT_EQ(PM_CONTROL_RESULT_NOT_READY, step(&output));
	input.referenced = true;
	input.faulted = true;
	EXPECT_EQ(PM_CONTROL_RESULT_NOT_READY, step(&output));
}

TEST_F(PmControlTest, InvalidDtAndNonfiniteInputsDoNotMutateState) {
	pm_control_output_t output = {};
	ASSERT_EQ(PM_CONTROL_RESULT_OK, step(&output));
	pm_control_t before = control;
	const float invalid_steps[] = {0.0f, -0.01f, 0.1001f, NAN, INFINITY};
	for (unsigned int index = 0;
			index < sizeof(invalid_steps) / sizeof(invalid_steps[0]); index++) {
		EXPECT_EQ(PM_CONTROL_RESULT_INVALID_DT,
				step(&output, invalid_steps[index]));
		EXPECT_FLOAT_EQ(before.velocity_integral_amps,
				control.velocity_integral_amps);
		EXPECT_FLOAT_EQ(before.filtered_measured_velocity_counts_per_second,
				control.filtered_measured_velocity_counts_per_second);
	}
	input.measured_velocity_counts_per_second = NAN;
	EXPECT_EQ(PM_CONTROL_RESULT_NUMERIC_ERROR, step(&output));
	EXPECT_FLOAT_EQ(before.velocity_integral_amps,
			control.velocity_integral_amps);
}

TEST_F(PmControlTest, OverflowingPositionDifferenceIsRejected) {
	pm_control_output_t output = {};
	input.commanded_position_counts = INT64_MAX;
	input.measured_position_counts = INT64_MIN;
	EXPECT_EQ(PM_CONTROL_RESULT_NUMERIC_ERROR, step(&output));
	EXPECT_FALSE(output.output_valid);
}

TEST_F(PmControlTest, InvalidConfigurationAndNullArgumentsAreRejected) {
	pm_control_output_t output = {};
	EXPECT_EQ(PM_CONTROL_RESULT_INVALID_ARGUMENT,
			pm_control_init(NULL, &config));
	pm_control_config_t invalid = config;
	invalid.current_limit_amps = 0.0f;
	EXPECT_EQ(PM_CONTROL_RESULT_INVALID_ARGUMENT,
			pm_control_init(&control, &invalid));
	invalid = config;
	invalid.velocity_filter_time_constant_seconds = -1.0f;
	EXPECT_EQ(PM_CONTROL_RESULT_INVALID_ARGUMENT,
			pm_control_init(&control, &invalid));
	EXPECT_EQ(PM_CONTROL_RESULT_INVALID_ARGUMENT,
			pm_control_reset(NULL, 0.0f));
	EXPECT_EQ(PM_CONTROL_RESULT_INVALID_ARGUMENT,
			pm_control_step(&control, &input, 0.01f, NULL));
	EXPECT_EQ(PM_CONTROL_RESULT_INVALID_ARGUMENT,
			pm_control_step(NULL, &input, 0.01f, &output));
}