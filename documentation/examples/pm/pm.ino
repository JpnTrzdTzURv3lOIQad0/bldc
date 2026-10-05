#include "PmSerialClient.h"

#define PM_EXAMPLE 1

#define PM_SERIAL Serial

#ifdef PM_DEBUG_PORT
#define PM_LOG(message) PM_DEBUG_PORT.println(message)
#else
#define PM_LOG(message) do { } while (0)
#endif

static const uint32_t LEASE_US = 1000000;
static const uint16_t AXIS = 0;
static const int64_t MOVE_COUNTS = 100;
static const int64_t VELOCITY_COUNTS_PER_SECOND = 100;

PmSerialClient pm(PM_SERIAL);
PmCapabilities capabilities;
bool leaseHeld = false;
uint32_t lastRenewMs = 0;
uint32_t activeLeaseUs = 0;
uint32_t renewIntervalMs = 250;

static void fail(const char *message) {
	PM_LOG(message);
#ifdef PM_DEBUG_PORT
	PM_DEBUG_PORT.print("PM result: ");
	PM_DEBUG_PORT.println(pm.lastResult());
#endif
	while (true) {
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		delay(250);
	}
}

static bool submit(uint8_t type, int64_t value = 0, uint32_t speed = 0,
		uint32_t currentMilliAmps = 0, uint8_t flags = 0) {
	bool received = pm.command(type, value, speed, currentMilliAmps, flags);
	if (!received) received = pm.retryLastCommand();
	return received && pm.lastResult() == PM_RESULT_ACCEPTED_PENDING;
}

static bool renewIfDue() {
	if ((uint32_t)(millis() - lastRenewMs) < renewIntervalMs) return true;
	if (!pm.renew(activeLeaseUs)) return false;
	lastRenewMs = millis();
	return true;
}

static bool waitForTerminal(uint32_t commandId, uint32_t timeoutMs,
		PmStatus &status, bool allowLatchedFault = false) {
	uint32_t started = millis();
	while ((uint32_t)(millis() - started) < timeoutMs) {
		if (!renewIfDue() || !pm.readStatus(status)) return false;
		if (status.faultCode && !allowLatchedFault) return false;
		if (status.terminalCommandId == commandId) {
			if (status.terminalResult == PM_TERMINAL_COMPLETED) return true;
			if (status.terminalResult != 0) return false;
		}
		delay(20);
	}
	return false;
}

static bool waitForAtTargetVelocity(uint32_t timeoutMs, PmStatus &status) {
	uint32_t started = millis();
	while ((uint32_t)(millis() - started) < timeoutMs) {
		if (!renewIfDue() || !pm.readStatus(status) || status.faultCode) return false;
		if (status.atTargetVelocity) return true;
		delay(20);
	}
	return false;
}

static bool claimAxis() {
	activeLeaseUs = capabilities.maximumLeaseUs < LEASE_US ?
			capabilities.maximumLeaseUs : LEASE_US;
	if (activeLeaseUs == 0) return false;
	renewIntervalMs = activeLeaseUs / 3000;
	if (renewIntervalMs == 0) renewIntervalMs = 1;
	uint32_t session = ((uint32_t)random(1, 0x7FFFFFFF) << 1) ^
			(uint32_t)random(1, 0x7FFFFFFF) ^ (uint32_t)micros();
	if (session == 0) session = 1;
	pm.begin(session, AXIS);
	if (!pm.claim(activeLeaseUs)) return false;
	leaseHeld = true;
	lastRenewMs = millis();
	return true;
}

static void discoveryExample() {
	PmStatus status;
	if (!pm.readCapabilities(capabilities) || !pm.readStatus(status)) {
		fail("PM discovery/status request failed");
	}
	PM_LOG("PM capabilities and status received");
}

static void homeAndRelativeMoveExample() {
	PmStatus status;
	if (!pm.readCapabilities(capabilities)) fail("PM capability request failed");
	uint32_t required = PM_CAP_POSITION | PM_CAP_HOME_SWITCH;
	if ((capabilities.flags & required) != required) {
		fail("Qualified position and switch-home capability required");
	}
	if (!claimAxis() || !pm.readStatus(status)) fail("Owner claim failed");
	if (status.faultCode || status.enabled || status.lifecycle == PM_LIFECYCLE_FAULTED) {
		fail("Axis must be healthy and disabled");
	}
	if (!submit(PM_COMMAND_ENABLE)) fail("Enable rejected");
	uint32_t id = pm.lastCommandId();
	if (!waitForTerminal(id, 5000, status)) fail("Enable did not complete");

	if (!submit(PM_COMMAND_HOME)) fail("Switch homing rejected");
	id = pm.lastCommandId();
	if (!waitForTerminal(id, 120000, status) || !status.referenced) {
		fail("Switch homing did not complete successfully");
	}
	if (status.commandedEndpoint > INT64_MAX - MOVE_COUNTS ||
			status.commandedEndpoint + MOVE_COUNTS > capabilities.maximumPosition ||
			status.commandedEndpoint + MOVE_COUNTS < capabilities.minimumPosition) {
		fail("Relative target exceeds commissioned position bounds");
	}
	if (!submit(PM_COMMAND_MOVE_RELATIVE, MOVE_COUNTS)) fail("Relative move rejected");
	id = pm.lastCommandId();
	if (!waitForTerminal(id, 120000, status)) fail("Move did not settle");
	PM_LOG("Move completed; axis remains enabled in position hold");
}

static void velocityExample() {
	PmStatus status;
	if (!pm.readCapabilities(capabilities)) fail("PM capability request failed");
	uint32_t required = PM_CAP_POSITION | PM_CAP_VELOCITY;
	if ((capabilities.flags & required) != required ||
			(uint64_t)VELOCITY_COUNTS_PER_SECOND > capabilities.maximumVelocity) {
		fail("Qualified position and velocity capability required");
	}
	if (!claimAxis() || !pm.readStatus(status)) fail("Owner claim failed");
	if (status.faultCode || status.enabled || !status.referenced) {
		fail("Axis must be healthy, referenced, and disabled");
	}
	if (!submit(PM_COMMAND_ENABLE)) fail("Enable rejected");
	uint32_t id = pm.lastCommandId();
	if (!waitForTerminal(id, 5000, status)) fail("Enable did not complete");
	if (!submit(PM_COMMAND_VELOCITY, VELOCITY_COUNTS_PER_SECOND)) {
		fail("Velocity request rejected");
	}
	uint32_t started = millis();
	while ((uint32_t)(millis() - started) < 2000) {
		if (!renewIfDue() || !pm.readStatus(status) || status.faultCode) {
			fail("Velocity run lost status or lease");
		}
		delay(20);
	}

	if (!submit(PM_COMMAND_VELOCITY, 0, 0, 0, 1)) fail("Zero-velocity update rejected");
	if (!waitForAtTargetVelocity(30000, status)) fail("Zero speed was not reached");
	if (!submit(PM_COMMAND_STOP_DECELERATED)) fail("Controlled stop rejected");
	id = pm.lastCommandId();
	if (!waitForTerminal(id, 30000, status)) fail("Controlled stop did not settle");
	if (!submit(PM_COMMAND_DISABLE)) fail("Disable rejected");
	id = pm.lastCommandId();
	if (!waitForTerminal(id, 5000, status) || status.enabled) fail("Disable did not complete");
	PM_LOG("Velocity ramp, stop, and disable completed");
}

static void acknowledgedFaultClearExample() {
	PmStatus status;
	if (!pm.readCapabilities(capabilities) || !pm.readStatus(status)) {
		fail("PM capability/status request failed");
	}
	if (!status.faultCode || status.lifecycle != PM_LIFECYCLE_FAULTED) {
		fail("Run only when an existing fault cause has been resolved");
	}
	if (!claimAxis()) fail("Owner claim failed");
	if (!submit(PM_COMMAND_CLEAR_FAULT, 0, 0, 0, 2)) {
		fail("Clear-fault rejected; check feedback health and acknowledgement");
	}
	uint32_t id = pm.lastCommandId();
	if (!waitForTerminal(id, 5000, status, true) || status.faultCode || status.enabled) {
		fail("Fault did not clear into the disabled state");
	}
	PM_LOG("Fault cleared; axis remains disabled");
}

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	PM_SERIAL.begin(115200);
#ifdef PM_DEBUG_PORT
	PM_DEBUG_PORT.begin(115200);
#endif
	delay(500);
	randomSeed((uint32_t)micros() ^ (uint32_t)analogRead(A0));

#if PM_EXAMPLE < 1 || PM_EXAMPLE > 4
#error "Set PM_EXAMPLE to 1, 2, 3, or 4"
#elif PM_EXAMPLE == 1
	discoveryExample();
#elif PM_EXAMPLE == 2
	homeAndRelativeMoveExample();
#elif PM_EXAMPLE == 3
	velocityExample();
#elif PM_EXAMPLE == 4
	acknowledgedFaultClearExample();
#endif
	digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
	if (leaseHeld && !renewIfDue()) fail("Owner lease renewal failed");
	delay(10);
}
