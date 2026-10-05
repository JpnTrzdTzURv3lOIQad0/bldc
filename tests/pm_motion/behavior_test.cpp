#include <gtest/gtest.h>
#include <climits>
#include <cmath>
#include <cstring>
#include <vector>
extern "C" {
#include "motor/pm_protocol.h"
#include "comm/packet.h"
}

class Runtime : public testing::Test {
protected:
	pm_runtime_t r{};
	pm_runtime_config_t c{};
	pm_runtime_sample_t sample{};
	pm_protocol_t protocol{};
	uint32_t now = 1000, generation = 0, id = 0;
	unsigned depth = 0;
	bool revoke_between_compute_and_commit = false;
	static uintptr_t lock(void *p) { auto *t = static_cast<Runtime *>(p); return t->depth++; }
	static void unlock(void *p, uintptr_t prior) {
		auto *t = static_cast<Runtime *>(p);
		EXPECT_EQ(prior + 1, t->depth); t->depth = static_cast<unsigned>(prior);
		if (t->revoke_between_compute_and_commit && prior == 0) {
			t->revoke_between_compute_and_commit = false;
			pm_runtime_revoke(&t->r, PM_FAULT_BACKEND);
		}
	}
	void SetUp() override {
		c.qualified = true;
		c.capabilities = PM_CAP_POSITION | PM_CAP_VELOCITY | PM_CAP_CURRENT |
			PM_CAP_HOME_SWITCH | PM_CAP_FOLLOW | PM_CAP_PRESET | PM_CAP_TELEMETRY;
		c.lease_max_us = 10000000; c.output_max_age_us = 20000; c.source_max_age_us = 10000;
		c.minimum_position = -(INT64_C(1) << 55); c.maximum_position = INT64_C(1) << 55;
		c.acceleration = 1000.0f; c.deceleration = 1000.0f;
		c.feedback.source_id = 1; c.feedback.counts_per_revolution = 10000;
		c.feedback.max_counts_per_second = 10000;
		c.feedback.max_sample_gap_us = 10000; c.feedback.max_sample_age_us = 10000;
		c.control.max_velocity_counts_per_second = 1000.0f; c.control.current_limit_amps = 2.0f;
		c.control.max_dt_seconds = 0.01f; c.control.following_error_limit_counts = 1000;
		c.control.position_gain_per_second = 10.0f;
		c.control.velocity_kp_amps_per_count_per_second = 0.1f;
		c.settle.position_tolerance_counts = 1; c.settle.velocity_tolerance_counts_per_second = 5.0f;
		c.settle.settle_duration_ms = 2; c.settle.timeout_ms = 50;
		c.home.seek_velocity = 100.0f; c.home.approach_velocity = 10.0f;
		c.home.current_limit_amps = 1.0f; c.home.max_travel = 100;
		c.home.timeout_us = 50000; c.home.reference = 0; c.home.offset = 0;
		c.behavior.position_count = 16; c.behavior.increment_count = 4; c.behavior.velocity_count = 4;
		for (int i = 0; i < 16; ++i) c.behavior.positions[i] = i * 10;
		for (int i = 0; i < 4; ++i) { c.behavior.increments[i] = i + 1; c.behavior.velocities[i] = i * 20; }
		c.behavior.primary_speed = 100.0f; c.behavior.alternate_speed = 20.0f;
		ASSERT_TRUE(pm_runtime_init(&r, &c, lock, unlock, this));
		pm_protocol_init(&protocol, &r);
		sample.has_sample = true; sample.home_valid = true; sample.fault_clear = true;
		sample.feedback.source_id = 1; sample.feedback.sensor_healthy = true; sample.feedback.commutation_valid = true;
		sample.feedback.reset_epoch = 1;
		advance();
		ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_claim(&r, 42, 9000000, now, &generation));
	}
	void advance(uint32_t delta = 1000, uint32_t raw = 0) {
		now += delta; sample.feedback.sequence++; sample.feedback.acquisition_time_us = now;
		sample.feedback.position_counts = raw; pm_runtime_tick(&r, &sample, now);
	}
	pm_command_t command(pm_command_type_t type, int64_t value = 0, bool replace = false) {
		pm_command_t cmd{}; cmd.owner_session = 42; cmd.owner_generation = generation;
		cmd.command_id = ++id; cmd.type = type; cmd.value_ticks = value; cmd.replace_active = replace;
		return cmd;
	}
	void send(pm_command_type_t type, int64_t value = 0, bool replace = false) {
		auto cmd = command(type, value, replace); ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_submit(&r, &cmd, now)); advance();
	}
	void ready(int64_t reference = 0) { send(PM_COMMAND_SET_REFERENCE, reference); send(PM_COMMAND_ENABLE); }
	static void put32(uint8_t *b, uint32_t v) { for (int i = 3; i >= 0; --i) { b[i] = static_cast<uint8_t>(v); v >>= 8; } }
	std::vector<uint8_t> packet(uint8_t op, size_t n = 16) {
		std::vector<uint8_t> b(n); b[0] = PM_PROTOCOL_VERSION; b[1] = op;
		put32(&b[4], 42); put32(&b[8], generation); put32(&b[12], ++id); return b;
	}
	pm_result_t exchange(const std::vector<uint8_t> &b) {
		uint8_t response[PM_PROTOCOL_MAX_RESPONSE];
		size_t n = pm_protocol_process(&protocol, b.data(), b.size(), response, sizeof(response), now);
		EXPECT_GE(n, 24U); EXPECT_LE(n, sizeof(response)); return static_cast<pm_result_t>(response[16]);
	}
};

TEST_F(Runtime, FaultSurvivesDisableRevokeReclaimAndRequiresBackendHealth) {
	ready(); pm_runtime_revoke(&r, PM_FAULT_BACKEND);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_claim(&r, 42, 100000, now, &generation)); id = 0;
	send(PM_COMMAND_DISABLE); send(PM_COMMAND_ENABLE);
	EXPECT_EQ(PM_LIFECYCLE_FAULTED, r.engine.axis.status.lifecycle);
	auto clear = command(PM_COMMAND_CLEAR_FAULT); clear.acknowledged = true;
	clear.feedback_valid = true; clear.fault_inputs_clear = true;
	sample.fault_clear = false;
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_submit(&r, &clear, now)); advance();
	EXPECT_EQ(PM_FAULT_BACKEND, r.engine.axis.status.fault_code);
	sample.fault_clear = true; clear.command_id = ++id;
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_submit(&r, &clear, now)); advance();
	EXPECT_EQ(0U, r.engine.axis.status.fault_code); EXPECT_FALSE(r.engine.axis.status.enabled);
}
TEST_F(Runtime, AbortPreemptsFullMailboxAndNoQueuedMoveRestarts) {
	ready(); auto move = command(PM_COMMAND_MOVE_RELATIVE, 10);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_submit(&r, &move, now));
	auto abort = command(PM_COMMAND_ABORT_RELEASE);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_submit(&r, &abort, now)); advance(); advance();
	EXPECT_FALSE(r.engine.axis.status.enabled); EXPECT_FALSE(r.engine.axis.status.command_active);
	EXPECT_EQ(PM_TERMINAL_ABORTED, r.engine.axis.status.last_terminal_result);
}
TEST_F(Runtime, StopPreemptsFullMailboxAndSettlesIntoHold) {
	ready(); auto move = command(PM_COMMAND_MOVE_RELATIVE, 10);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_submit(&r, &move, now));
	send(PM_COMMAND_STOP_DECELERATED); advance(); advance();
	EXPECT_EQ(PM_LIFECYCLE_HOLDING, r.engine.axis.status.lifecycle);
	EXPECT_EQ(0, r.engine.axis.status.last_commanded_endpoint_ticks);
}
TEST_F(Runtime, RevocationDuringCalculationDiscardsPublication) {
	ready(); send(PM_COMMAND_CURRENT, 1000); EXPECT_TRUE(r.output_valid);
	revoke_between_compute_and_commit = true; advance();
	EXPECT_FALSE(r.output_valid); EXPECT_EQ(PM_FAULT_BACKEND, r.engine.axis.status.fault_code);
	EXPECT_EQ(0U, r.engine.axis.status.owner_session);
}
TEST_F(Runtime, CurrentConsumerExpiresWithoutScheduler) {
	ready(); send(PM_COMMAND_CURRENT, 1000);
	float current = 0; ASSERT_TRUE(pm_runtime_consume(&r, now, &current)); EXPECT_FLOAT_EQ(1.0f, current);
	EXPECT_TRUE(pm_runtime_consume(&r, now + c.output_max_age_us, &current)); EXPECT_FLOAT_EQ(0, current);
	EXPECT_FALSE(pm_runtime_consume(&r, now + c.output_max_age_us + 1, &current));
	EXPECT_EQ(PM_FAULT_OUTPUT_EXPIRED, r.engine.axis.status.fault_code);
}
TEST_F(Runtime, LeaseLossInvalidatesQueuedWorkAndGeneration) {
	auto move = command(PM_COMMAND_ENABLE); ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_submit(&r, &move, now));
	float current; pm_runtime_consume(&r, now + 9000000, &current);
	EXPECT_EQ(PM_RESULT_STALE_OWNER, pm_runtime_submit(&r, &move, now + 9000000));
	EXPECT_FALSE(r.engine.axis.status.command_pending);
}
TEST_F(Runtime, ContinuousVelocityUpdatesReverseAndZeroWithoutCompleting) {
	ready(); send(PM_COMMAND_VELOCITY, 50);
	float first = r.engine.commanded_velocity;
	send(PM_COMMAND_VELOCITY, -20, true); EXPECT_LT(r.engine.commanded_velocity, first);
	send(PM_COMMAND_VELOCITY, 0, true); advance(); advance();
	EXPECT_TRUE(r.engine.axis.status.command_active); EXPECT_TRUE(r.engine.axis.status.at_target_velocity);
	EXPECT_EQ(PM_LIFECYCLE_MOVING, r.engine.axis.status.lifecycle);
	auto position = command(PM_COMMAND_MOVE_ABSOLUTE, 10, true);
	EXPECT_EQ(PM_RESULT_UNSUPPORTED, pm_runtime_submit(&r, &position, now));
}
TEST_F(Runtime, CurrentIsLimitedAndProtectedByTravelAndSpeed) {
	ready(); auto cmd = command(PM_COMMAND_CURRENT, 1500); cmd.current_limit_amps = 0.5f;
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_submit(&r, &cmd, now)); advance();
	EXPECT_FLOAT_EQ(0.5f, r.demand); EXPECT_TRUE(r.engine.output.current_saturated);
	sample.positive_limit = true; advance(); EXPECT_EQ(PM_FAULT_BOUNDS, r.engine.axis.status.fault_code); EXPECT_FALSE(r.output_valid);
}
TEST_F(Runtime, UnreferencedHomingAssignsReferenceAndRelativeEndpoint) {
	send(PM_COMMAND_ENABLE); EXPECT_FALSE(r.engine.axis.status.referenced);
	send(PM_COMMAND_HOME); EXPECT_TRUE(r.output_valid);
	sample.home_active = true; advance(); advance();
	sample.home_active = false; advance(); advance();
	EXPECT_EQ(PM_HOME_APPROACH, r.engine.home_state);
	sample.home_active = true; advance(); advance(); advance(); advance();
	ASSERT_EQ(PM_HOME_IDLE, r.engine.home_state);
	ASSERT_TRUE(r.engine.axis.status.referenced); ASSERT_TRUE(r.engine.feedback.referenced);
	EXPECT_EQ(0, r.engine.axis.status.last_commanded_endpoint_ticks);
	send(PM_COMMAND_MOVE_RELATIVE, 5); EXPECT_EQ(5, r.engine.axis.status.active_endpoint_ticks);
}
TEST_F(Runtime, InitiallyActiveHomeBacksOffAndTimesOutWithoutSwitchRelease) {
	send(PM_COMMAND_ENABLE); sample.home_active = true; send(PM_COMMAND_HOME);
	EXPECT_EQ(PM_HOME_BACKOFF, r.engine.home_state); EXPECT_LT(r.engine.commanded_velocity, 0.0f);
	for (int i = 0; i < 51; ++i) advance();
	EXPECT_EQ(PM_FAULT_HOME, r.engine.axis.status.fault_code); EXPECT_FALSE(r.engine.feedback.referenced);
}
TEST_F(Runtime, AbortDuringHomeInvalidatesReference) {
	ready(); send(PM_COMMAND_HOME); send(PM_COMMAND_ABORT_RELEASE);
	EXPECT_FALSE(r.engine.axis.status.referenced); EXPECT_FALSE(r.engine.feedback.referenced); EXPECT_FALSE(r.output_valid);
}
TEST_F(Runtime, SensorLossResetAndSchedulerGapCannotKeepCurrentAlive) {
	ready(); send(PM_COMMAND_CURRENT, 1000); sample.feedback.reset_epoch++; advance();
	EXPECT_EQ(PM_FAULT_FEEDBACK, r.engine.axis.status.fault_code); EXPECT_FALSE(r.output_valid);
}
TEST_F(Runtime, FrozenAcquisitionDoesNotRefreshFeedback) {
	ready(); send(PM_COMMAND_CURRENT, 1000); sample.has_sample = false;
	for (int i = 0; i < 12; ++i) advance();
	EXPECT_EQ(PM_FAULT_FEEDBACK, r.engine.axis.status.fault_code); EXPECT_FALSE(r.output_valid);
}
TEST_F(Runtime, SchedulerGapFaults) {
	ready(); advance(11000); EXPECT_EQ(PM_FAULT_TIMING, r.engine.axis.status.fault_code);
	EXPECT_EQ(0U, r.engine.axis.status.owner_session);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_claim(&r, 42, 100000, now, &generation));
	send(PM_COMMAND_ENABLE);
	EXPECT_EQ(PM_FAULT_TIMING, r.engine.axis.status.fault_code);
	EXPECT_FALSE(r.engine.axis.status.enabled);
	auto clear = command(PM_COMMAND_CLEAR_FAULT);
	clear.acknowledged = true;
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_submit(&r, &clear, now));
	advance();
	EXPECT_EQ(0U, r.engine.axis.status.fault_code);
	EXPECT_FALSE(r.engine.axis.status.enabled);
}
TEST_F(Runtime, LargeCoordinateRelativeMotionPreservesSingleCount) {
	int64_t base = INT64_C(1) << 50; ready(base); send(PM_COMMAND_MOVE_RELATIVE, 1);
	EXPECT_EQ(base + 1, r.engine.profile.endpoint);
	for (int i = 0; i < 80 && r.engine.axis.status.command_active; ++i) advance(1000, 1);
	EXPECT_EQ(base + 1, r.engine.axis.status.last_commanded_endpoint_ticks);
	EXPECT_EQ(PM_TERMINAL_COMPLETED, r.engine.axis.status.last_terminal_result);
}
TEST_F(Runtime, DuplicateRelativeAndChangedPayloadDoNotExecuteAgain) {
	ready(); auto cmd = command(PM_COMMAND_MOVE_RELATIVE, 10);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_submit(&r, &cmd, now)); advance();
	EXPECT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_submit(&r, &cmd, now));
	EXPECT_FALSE(r.engine.axis.status.command_pending);
	cmd.value_ticks = 20; EXPECT_EQ(PM_RESULT_INVALID, pm_runtime_submit(&r, &cmd, now));
	EXPECT_EQ(10, r.engine.axis.status.active_endpoint_ticks);
}
TEST_F(Runtime, EncoderFollowUsesIndependentSourceAndStopsOnLoss) {
	ready(); pm_input_t input{}; pm_input_config_t cfg{};
	cfg.type = PM_INPUT_QUADRATURE; cfg.source_id = 2; cfg.max_age_us = 10000; cfg.minimum = -1000; cfg.maximum = 1000;
	ASSERT_TRUE(pm_input_init(&input, &cfg));
	pm_input_event_t event{2, 42, generation, 1, now, 10, true};
	ASSERT_TRUE(pm_input_update(&input, &event, 42, generation, now));
	pm_behavior_request_t req{}; req.type = PM_BEHAVIOR_FOLLOW_VELOCITY;
	auto cmd = command(PM_COMMAND_VELOCITY);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_behavior(&r, &req, &input, &cmd, now)); advance();
	EXPECT_TRUE(r.engine.following);
	for (int i = 0; i < 20; ++i) advance();
	EXPECT_FALSE(r.engine.following); EXPECT_EQ(PM_LIFECYCLE_HOLDING, r.engine.axis.status.lifecycle);
}
TEST_F(Runtime, EveryExampleHasExplicitCapabilityOutcome) {
	for (int i = 0; i < PM_EX_COUNT; ++i) {
		auto e = static_cast<pm_example_t>(i);
		bool hard = e == PM_EX_ABS4_HARDSTOP || e == PM_EX_ABS16_HARDSTOP || e == PM_EX_INCREMENT4_HARDSTOP || e == PM_EX_USER_HOME_HARDSTOP;
		auto expected = hard ? PM_UNSUPPORTED_HARDSTOP : e == PM_EX_DUAL_AXIS ? PM_UNSUPPORTED_DUAL_AXIS :
			e == PM_EX_ASG_TORQUE ? PM_UNSUPPORTED_TORQUE_CALIBRATION : PM_UNSUPPORTED_NONE;
		EXPECT_EQ(expected, pm_behavior_example(e, c.capabilities)) << i;
		EXPECT_NE(PM_UNSUPPORTED_NONE, pm_behavior_example(e, 0)) << i;
	}
}
TEST_F(Runtime, PresetsIncrementRepetitionsManualVelocityAndAlternateSpeedMap) {
	ready(); pm_behavior_request_t req{}; req.type = PM_BEHAVIOR_INCREMENT; req.selection = 2; req.repetitions = 4;
	auto cmd = command(PM_COMMAND_MOVE_RELATIVE);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_behavior(&r, &req, nullptr, &cmd, now)); advance();
	EXPECT_EQ(12, r.engine.axis.status.active_endpoint_ticks);
	send(PM_COMMAND_ABORT_RELEASE); send(PM_COMMAND_ENABLE);
	req.type = PM_BEHAVIOR_PULSE_BURST; req.value = 5; req.alternate_speed = true; cmd = command(PM_COMMAND_MOVE_RELATIVE);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_behavior(&r, &req, nullptr, &cmd, now)); advance();
	EXPECT_FLOAT_EQ(20.0f, r.engine.speed_limit);
}
TEST_F(Runtime, UsbRejectsMalformedLengthsVersionsAxesAndFlags) {
	auto b = packet(PM_USB_COMMAND, 34); b[16] = PM_COMMAND_ENABLE;
	for (size_t n = 1; n < b.size(); ++n) { auto short_b = b; short_b.resize(n); EXPECT_EQ(PM_RESULT_INVALID, exchange(short_b)); }
	auto bad = b; bad.resize(PM_PROTOCOL_MAX_REQUEST + 1); EXPECT_EQ(PM_RESULT_INVALID, exchange(bad));
	bad = b; bad[0]++; EXPECT_EQ(PM_RESULT_INVALID, exchange(bad));
	bad = b; bad[3] = 1; EXPECT_EQ(PM_RESULT_INVALID, exchange(bad));
	bad = b; bad[17] = 128; EXPECT_EQ(PM_RESULT_INVALID, exchange(bad));
	bad = b; bad[8]++; EXPECT_EQ(PM_RESULT_STALE_OWNER, exchange(bad));
}
TEST_F(Runtime, UsbBehaviorRetryDoesNotReapplyManualVelocityDelta) {
	ready(); auto b = packet(PM_USB_BEHAVIOR, 36); b[16] = PM_BEHAVIOR_MANUAL_VELOCITY; b[31] = 10;
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, exchange(b)); advance();
	EXPECT_EQ(10, r.engine.axis.status.active_value);
	EXPECT_EQ(PM_RESULT_ACCEPTED_PENDING, exchange(b)); advance();
	EXPECT_EQ(10, r.engine.axis.status.active_value); EXPECT_FALSE(r.engine.axis.status.command_pending);
	b[31] = 11; EXPECT_EQ(PM_RESULT_INVALID, exchange(b));
}
TEST_F(Runtime, StatusDoesNotRenewLeaseAndReportsUnsupportedBoard) {
	auto b = packet(PM_USB_STATUS); uint32_t deadline = r.lease_deadline_us;
	EXPECT_EQ(PM_RESULT_ACCEPTED_PENDING, exchange(b)); EXPECT_EQ(deadline, r.lease_deadline_us);
	pm_runtime_revoke(&r, 0); c.qualified = false;
	ASSERT_TRUE(pm_runtime_init(&r, &c, lock, unlock, this)); pm_protocol_init(&protocol, &r);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_claim(&r, 42, 100000, now, &generation));
	auto cmd = command(PM_COMMAND_ENABLE); EXPECT_EQ(PM_RESULT_UNSUPPORTED, pm_runtime_submit(&r, &cmd, now));
	pm_runtime_status_t status{}; pm_runtime_status(&r, &status); EXPECT_EQ(PM_CAP_TELEMETRY, status.capabilities);
}
TEST_F(Runtime, StopCannotDisplaceAbortAndMotionCannotDisplaceStop) {
	ready(); auto abort = command(PM_COMMAND_ABORT_RELEASE);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_submit(&r, &abort, now));
	auto stop = command(PM_COMMAND_STOP_DECELERATED);
	EXPECT_EQ(PM_RESULT_BUSY, pm_runtime_submit(&r, &stop, now));
	auto move = command(PM_COMMAND_MOVE_ABSOLUTE, 10);
	EXPECT_EQ(PM_RESULT_BUSY, pm_runtime_submit(&r, &move, now));
	advance(); EXPECT_FALSE(r.engine.axis.status.enabled);
}
TEST_F(Runtime, HomeTravelCurrentAndMissingSwitchFaultWithoutReference) {
	send(PM_COMMAND_ENABLE); send(PM_COMMAND_HOME);
	sample.measured_current_amps = 1.1f; advance();
	EXPECT_EQ(PM_FAULT_HOME, r.engine.axis.status.fault_code); EXPECT_FALSE(r.engine.feedback.referenced);
}
TEST_F(Runtime, HomeMissingSwitchIsNotTreatedAsInactive) {
	send(PM_COMMAND_ENABLE); sample.home_valid = false; send(PM_COMMAND_HOME);
	EXPECT_EQ(PM_FAULT_HOME, r.engine.axis.status.fault_code); EXPECT_FALSE(r.output_valid);
}
TEST_F(Runtime, PositionSettleTimeoutCannotResumeAfterFreshFeedbackReturns) {
	ready(); send(PM_COMMAND_MOVE_RELATIVE, 10);
	for (int i = 0; i < 300; ++i) advance();
	EXPECT_EQ(PM_FAULT_SETTLE_TIMEOUT, r.engine.axis.status.fault_code);
	advance(1000, 10); EXPECT_FALSE(r.output_valid); EXPECT_FALSE(r.engine.axis.status.enabled);
}
TEST_F(Runtime, RelativeBoundsRejectionPreservesActiveEndpoint) {
	ready(); send(PM_COMMAND_MOVE_ABSOLUTE, 5);
	auto cmd = command(PM_COMMAND_MOVE_RELATIVE, INT64_MAX, true);
	EXPECT_EQ(PM_RESULT_INVALID, pm_runtime_submit(&r, &cmd, now));
	EXPECT_EQ(5, r.engine.axis.status.active_endpoint_ticks); EXPECT_EQ(0U, r.engine.axis.status.fault_code);
}
TEST_F(Runtime, EvictedDuplicateIsRejectedInsteadOfReexecuted) {
	auto first = command(PM_COMMAND_SET_REFERENCE, 1);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_runtime_submit(&r, &first, now)); advance();
	for (int i = 0; i < PM_REPLAY_SLOTS + 1; ++i) send(PM_COMMAND_SET_REFERENCE, i);
	EXPECT_EQ(PM_RESULT_INVALID, pm_runtime_submit(&r, &first, now));
	EXPECT_EQ(PM_REPLAY_SLOTS, r.engine.axis.status.last_commanded_endpoint_ticks);
}
TEST_F(Runtime, UsbClaimRenewAndExpiredRetry) {
	pm_runtime_revoke(&r, 0);
	auto claim = packet(PM_USB_CLAIM, 20); put32(&claim[8], 0); put32(&claim[16], 10000);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, exchange(claim));
	generation = r.engine.axis.status.owner_generation;
	uint32_t deadline = r.lease_deadline_us; now += 100;
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, exchange(claim)); EXPECT_EQ(deadline, r.lease_deadline_us);
	auto renew = packet(PM_USB_RENEW, 20); put32(&renew[16], 10000);
	ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, exchange(renew)); EXPECT_GT(r.lease_deadline_us, deadline);
	now = r.lease_deadline_us; EXPECT_EQ(PM_RESULT_STALE_OWNER, exchange(renew));
}
TEST_F(Runtime, UsbRejectsNumericAndTruncatedRequestsWithoutMutatingState) {
	ready(); auto command_packet = packet(PM_USB_COMMAND, 34);
	command_packet[16] = PM_COMMAND_CURRENT; std::memset(&command_packet[18], 0x7f, 8);
	EXPECT_EQ(PM_RESULT_INVALID, exchange(command_packet));
	uint8_t response[PM_PROTOCOL_MAX_RESPONSE + 2]; std::memset(response, 0x55, sizeof(response));
	uint8_t data[256]{};
	for (size_t n = 0; n <= sizeof(data); ++n) {
		size_t count = pm_protocol_process(&protocol, data, n, response + 1, PM_PROTOCOL_MAX_RESPONSE, now);
		EXPECT_EQ(24U, count); EXPECT_EQ(0x55, response[0]); EXPECT_EQ(0x55, response[sizeof(response)-1]);
	}
	EXPECT_EQ(0U, pm_protocol_process(&protocol, data, 16, response, 1, now));
	EXPECT_FALSE(r.engine.axis.status.command_active);
}
TEST_F(Runtime, AllPresetAndVelocitySelectionsAndIncrementOverflow) {
	pm_axis_status_t status{}; pm_command_t cmd{}; float speed;
	pm_behavior_request_t req{}; req.type = PM_BEHAVIOR_PRESET;
	for (unsigned i = 0; i < 16; ++i) {
		req.selection = i;
		ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_behavior_map(&c.behavior, c.capabilities, &req, &status, &cmd, &speed));
		EXPECT_EQ(i * 10, cmd.value_ticks);
	}
	req.selection = 16; EXPECT_EQ(PM_RESULT_INVALID, pm_behavior_map(&c.behavior, c.capabilities, &req, &status, &cmd, &speed));
	req.type = PM_BEHAVIOR_SELECTED_VELOCITY;
	for (unsigned i = 0; i < 4; ++i) {
		req.selection = i;
		ASSERT_EQ(PM_RESULT_ACCEPTED_PENDING, pm_behavior_map(&c.behavior, c.capabilities, &req, &status, &cmd, &speed));
		EXPECT_EQ(i * 20, cmd.value_ticks);
	}
	c.behavior.increments[0] = INT64_MAX; req.type = PM_BEHAVIOR_INCREMENT; req.selection = 0; req.repetitions = 2;
	EXPECT_EQ(PM_RESULT_INVALID, pm_behavior_map(&c.behavior, c.capabilities, &req, &status, &cmd, &speed));
}

static std::vector<uint8_t> framed, decoded;
static void save_frame(unsigned char *b, unsigned int n) { framed.assign(b, b + n); }
static void save_payload(unsigned char *b, unsigned int n) { decoded.assign(b, b + n); }
TEST(PacketFraming, PmSizesLongFramesCrcTruncationAndRecovery) {
	PACKET_STATE_t tx{}, rx{};
	packet_init(save_frame, nullptr, &tx); packet_init(nullptr, save_payload, &rx);
	for (unsigned n : {17U, 35U, 37U, 129U, 255U, 256U, 512U}) {
		std::vector<uint8_t> payload(n, 0xA5); payload[0] = 160;
		packet_send_packet(payload.data(), n, &tx); decoded.clear(); packet_reset(&rx);
		for (size_t i = 0; i + 1 < framed.size(); ++i) packet_process_byte(framed[i], &rx);
		EXPECT_TRUE(decoded.empty()); packet_process_byte(framed.back(), &rx); EXPECT_EQ(payload, decoded);
		decoded.clear(); packet_reset(&rx); framed[framed.size() - 2] ^= 1;
		for (uint8_t byte : framed) packet_process_byte(byte, &rx);
		EXPECT_TRUE(decoded.empty());
		packet_reset(&rx); packet_send_packet(payload.data(), n, &tx);
		for (uint8_t byte : framed) packet_process_byte(byte, &rx);
		EXPECT_EQ(payload, decoded);
	}
}
TEST(Input, AnalogFilteringAndOutOfOrderTimestamps) {
	pm_input_config_t c{}; c.type = PM_INPUT_ANALOG; c.source_id = 1;
	c.minimum = 0; c.maximum = 100; c.max_age_us = 10000; c.filter_time_constant_us = 1000;
	pm_input_t p{}; ASSERT_TRUE(pm_input_init(&p, &c));
	pm_input_event_t e{1, 1, 1, 1, 1000, 0, true}; ASSERT_TRUE(pm_input_update(&p, &e, 1, 1, 1000));
	e.sequence++; e.acquired_us = 2000; e.value = 100; ASSERT_TRUE(pm_input_update(&p, &e, 1, 1, 2000)); EXPECT_EQ(50, p.value);
	e.sequence++; e.acquired_us = 1500; EXPECT_FALSE(pm_input_update(&p, &e, 1, 1, 2000)); EXPECT_EQ(50, p.value);
	e.healthy = false; EXPECT_FALSE(pm_input_update(&p, &e, 1, 1, 2000)); EXPECT_FALSE(pm_input_fresh(&p, 2000));
}
TEST(Profile, IntegerOriginRetargetStopAndLimits) {
	for (int sign : {-1, 1}) {
		int64_t base = sign * (INT64_C(1) << 54);
		pm_profile_t p{}; ASSERT_TRUE(pm_profile_start(&p, base, 0, base + sign, 10, 100, 100, 0.01f));
		int64_t position = base; float velocity = 0;
		for (int i = 0; i < 100; ++i) ASSERT_TRUE(pm_profile_step(&p, 0.01f, &position, &velocity));
		EXPECT_EQ(base + sign, position);
		ASSERT_TRUE(pm_profile_retarget(&p, base + 2 * sign, 10, 100, 100, 0.01f));
		for (int i = 0; i < 100; ++i) ASSERT_TRUE(pm_profile_step(&p, 0.01f, &position, &velocity));
		EXPECT_EQ(base + 2 * sign, position);
		ASSERT_TRUE(pm_profile_stop(&p, position, 10.0f * sign, 100, 0.01f));
		for (int i = 0; i < 100; ++i) ASSERT_TRUE(pm_profile_step(&p, 0.01f, &position, &velocity));
		EXPECT_EQ(base + 3 * sign, position);
	}
	pm_profile_t p{};
	EXPECT_FALSE(pm_profile_start(&p, INT64_MIN, 0, INT64_MAX, 10, 10, 10, 0.01f));
	EXPECT_FALSE(pm_profile_stop(&p, INT64_MAX, 100, 10, 0.01f));
}
TEST(Input, DebounceFreshnessOwnershipAndScaling) {
	pm_input_config_t cfg{}; cfg.type = PM_INPUT_DIGITAL; cfg.source_id = 1;
	cfg.minimum = 0; cfg.maximum = 1; cfg.max_age_us = 1000; cfg.debounce_us = 100;
	pm_input_t p{}; ASSERT_TRUE(pm_input_init(&p, &cfg));
	pm_input_event_t e{1, 2, 3, 1, UINT32_MAX - 49, 1, true};
	EXPECT_FALSE(pm_input_update(&p, &e, 2, 3, e.acquired_us));
	e.sequence++; e.acquired_us = 50;
	EXPECT_TRUE(pm_input_update(&p, &e, 2, 3, 50)); EXPECT_EQ(1, p.value);
	e.owner_generation++; EXPECT_FALSE(pm_input_update(&p, &e, 2, 3, 100));
	EXPECT_FALSE(pm_input_fresh(&p, 1051));
	cfg.type = PM_INPUT_ANALOG; cfg.debounce_us = 0; cfg.minimum = -100; cfg.maximum = 100;
	cfg.scale_numerator = 3; cfg.scale_denominator = 2; cfg.inverted = true;
	ASSERT_TRUE(pm_input_init(&p, &cfg)); e = {1, 2, 3, 1, 10, 10, true};
	ASSERT_TRUE(pm_input_update(&p, &e, 2, 3, 10)); EXPECT_EQ(-15, p.value);
}
TEST(FeedbackMicroseconds, SubMillisecondSamplesWrapAndRateValidation) {
	pm_feedback_config_t c{0, 1, 4096, 10000, 1000, 1000, false}; pm_feedback_t f{};
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK, pm_feedback_init(&f, &c));
	pm_feedback_sample_t s{0, 1, 1, UINT32_MAX - 49, 1, 100, true, true};
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK, pm_feedback_update(&f, &s, s.acquisition_time_us));
	s.sequence++; s.acquisition_time_us = 50; s.position_counts++;
	ASSERT_EQ(PM_FEEDBACK_RESULT_OK, pm_feedback_update(&f, &s, 50)); EXPECT_FLOAT_EQ(10000, f.velocity_counts_per_second);
	EXPECT_EQ(PM_FEEDBACK_RESULT_STALE, pm_feedback_update(&f, &s, 60));
	EXPECT_EQ(PM_FEEDBACK_RESULT_STALE, pm_feedback_check_freshness(&f, 1051));
	c.max_sample_gap_us = 1000000; EXPECT_EQ(PM_FEEDBACK_RESULT_INVALID_ARGUMENT, pm_feedback_init(&f, &c));
}
