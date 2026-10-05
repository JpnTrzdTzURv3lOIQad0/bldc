#ifndef PM_SERIAL_CLIENT_H
#define PM_SERIAL_CLIENT_H

#include <Arduino.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>

static const uint8_t PM_COMM_PACKET_ID = 160;
static const uint8_t PM_PROTOCOL_VERSION = 1;
static const size_t PM_PROTOCOL_MAX_REQUEST = 36;
static const size_t PM_PROTOCOL_MAX_RESPONSE = 128;
static const size_t PM_TRANSPORT_MAX_PAYLOAD = PM_PROTOCOL_MAX_RESPONSE + 1;
static const uint32_t PM_RESULT_ACCEPTED_PENDING = 0;

static const uint8_t PM_OP_CAPABILITIES = 0;
static const uint8_t PM_OP_CONFIGURATION = 1;
static const uint8_t PM_OP_CLAIM = 2;
static const uint8_t PM_OP_RENEW = 3;
static const uint8_t PM_OP_COMMAND = 4;
static const uint8_t PM_OP_STATUS = 5;

static const uint8_t PM_COMMAND_ENABLE = 0;
static const uint8_t PM_COMMAND_DISABLE = 1;
static const uint8_t PM_COMMAND_SET_REFERENCE = 2;
static const uint8_t PM_COMMAND_HOME = 3;
static const uint8_t PM_COMMAND_MOVE_ABSOLUTE = 4;
static const uint8_t PM_COMMAND_MOVE_RELATIVE = 5;
static const uint8_t PM_COMMAND_VELOCITY = 6;
static const uint8_t PM_COMMAND_STOP_DECELERATED = 7;
static const uint8_t PM_COMMAND_ABORT_RELEASE = 8;
static const uint8_t PM_COMMAND_CLEAR_FAULT = 9;
static const uint8_t PM_COMMAND_CURRENT = 10;

static const uint8_t PM_TERMINAL_COMPLETED = 1;
static const uint8_t PM_LIFECYCLE_FAULTED = 8;
static const uint32_t PM_CAP_POSITION = 1UL << 0;
static const uint32_t PM_CAP_VELOCITY = 1UL << 1;
static const uint32_t PM_CAP_HOME_SWITCH = 1UL << 3;

struct PmCapabilities {
	uint32_t flags;
	uint32_t maximumLeaseUs;
	uint32_t maximumOutputAgeUs;
	int64_t minimumPosition;
	int64_t maximumPosition;
	uint32_t maximumVelocity;
	uint32_t maximumCurrentMilliAmps;
};

struct PmStatus {
	uint32_t activeCommandId;
	uint32_t faultCode;
	int64_t measuredPosition;
	int64_t commandedEndpoint;
	uint32_t feedbackAgeUs;
	bool enabled;
	bool referenced;
	bool atTargetVelocity;
	bool feedbackValid;
	bool outputValid;
	bool commutationValid;
	int64_t measuredVelocity;
	int32_t currentDemandMilliAmps;
	uint8_t lifecycle;
	uint8_t terminalResult;
	uint32_t terminalCommandId;
};

class PmSerialClient {
public:
	explicit PmSerialClient(Stream &stream) : stream_(stream) {}

	void begin(uint32_t session, uint16_t axis = 0) {
		session_ = session ? session : 1;
		axis_ = axis;
		generation_ = 0;
		nextId_ = 1;
		lastCommandLength_ = 0;
		lastResult_ = 0xFF;
	}

	uint32_t generation() const { return generation_; }
	uint32_t lastCommandId() const { return lastCommandId_; }
	uint8_t lastResult() const { return lastResult_; }

	bool readCapabilities(PmCapabilities &caps) {
		if (!transact(PM_OP_CAPABILITIES, NULL, 0, false) ||
				lastResult_ != PM_RESULT_ACCEPTED_PENDING || responseLength_ < 82) {
			return false;
		}
		caps.flags = readU32(response_ + 24);
		caps.maximumLeaseUs = readU32(response_ + 28);
		caps.maximumOutputAgeUs = readU32(response_ + 32);
		caps.minimumPosition = readI64(response_ + 36);
		caps.maximumPosition = readI64(response_ + 44);
		caps.maximumVelocity = readU32(response_ + 52);
		caps.maximumCurrentMilliAmps = readU32(response_ + 56);
		return true;
	}

	bool claim(uint32_t leaseUs) {
		uint8_t payload[4];
		writeU32(payload, leaseUs);
		if (!transact(PM_OP_CLAIM, payload, sizeof(payload), true) ||
				lastResult_ != PM_RESULT_ACCEPTED_PENDING) {
			return false;
		}
		generation_ = readU32(response_ + 8);
		return generation_ != 0;
	}

	bool renew(uint32_t leaseUs) {
		uint8_t payload[4];
		writeU32(payload, leaseUs);
		return transact(PM_OP_RENEW, payload, sizeof(payload), false) &&
				lastResult_ == PM_RESULT_ACCEPTED_PENDING;
	}

	bool command(uint8_t type, int64_t value = 0, uint32_t speedLimit = 0,
			uint32_t currentLimitMilliAmps = 0, uint8_t flags = 0) {
		uint8_t payload[18] = {0};
		payload[0] = type;
		payload[1] = flags;
		writeI64(payload + 2, value);
		writeU32(payload + 10, speedLimit);
		writeU32(payload + 14, currentLimitMilliAmps);
		uint32_t id = allocateId();
		lastCommandId_ = id;
		lastCommandLength_ = makeEnvelope(PM_OP_COMMAND, payload, sizeof(payload), id,
				lastCommandEnvelope_);
		return transactEnvelope(lastCommandEnvelope_, lastCommandLength_, false);
	}

	bool retryLastCommand() {
		return lastCommandLength_ != 0 &&
				transactEnvelope(lastCommandEnvelope_, lastCommandLength_, false);
	}

	bool readStatus(PmStatus &status) {
		if (!transact(PM_OP_STATUS, NULL, 0, false) ||
				lastResult_ != PM_RESULT_ACCEPTED_PENDING || responseLength_ < 72) {
			return false;
		}
		status.activeCommandId = readU32(response_ + 24);
		status.faultCode = readU32(response_ + 28);
		status.measuredPosition = readI64(response_ + 32);
		status.commandedEndpoint = readI64(response_ + 40);
		status.feedbackAgeUs = readU32(response_ + 48);
		status.enabled = response_[52] != 0;
		status.referenced = response_[53] != 0;
		status.atTargetVelocity = response_[54] != 0;
		status.feedbackValid = response_[55] != 0;
		status.outputValid = response_[58] != 0;
		status.commutationValid = response_[59] != 0;
		status.measuredVelocity = readI64(response_ + 60);
		status.currentDemandMilliAmps = readI32(response_ + 68);
		status.lifecycle = response_[18];
		status.terminalResult = response_[19];
		status.terminalCommandId = readU32(response_ + 20);
		return true;
	}

private:
	Stream &stream_;
	uint32_t session_ = 0;
	uint32_t generation_ = 0;
	uint16_t axis_ = 0;
	uint32_t nextId_ = 1;
	uint32_t lastCommandId_ = 0;
	uint8_t lastResult_ = 0xFF;
	uint8_t response_[PM_PROTOCOL_MAX_RESPONSE] = {0};
	size_t responseLength_ = 0;
	uint8_t lastCommandEnvelope_[PM_PROTOCOL_MAX_REQUEST] = {0};
	size_t lastCommandLength_ = 0;

	static uint16_t crc16(const uint8_t *data, size_t length) {
		uint16_t crc = 0;
		for (size_t i = 0; i < length; ++i) {
			crc ^= (uint16_t)data[i] << 8;
			for (uint8_t bit = 0; bit < 8; ++bit) {
				crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
			}
		}
		return crc;
	}

	static uint32_t readU32(const uint8_t *p) {
		return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 |
				(uint32_t)p[2] << 8 | p[3];
	}

	static int32_t readI32(const uint8_t *p) {
		uint32_t bits = readU32(p);
		return bits <= INT32_MAX ? (int32_t)bits : -1 - (int32_t)(UINT32_MAX - bits);
	}

	static int64_t readI64(const uint8_t *p) {
		uint64_t bits = (uint64_t)readU32(p) << 32 | readU32(p + 4);
		return bits <= INT64_MAX ? (int64_t)bits : -1 - (int64_t)(UINT64_MAX - bits);
	}

	static void writeU32(uint8_t *p, uint32_t value) {
		p[0] = (uint8_t)(value >> 24);
		p[1] = (uint8_t)(value >> 16);
		p[2] = (uint8_t)(value >> 8);
		p[3] = (uint8_t)value;
	}

	static void writeI64(uint8_t *p, int64_t value) {
		uint64_t bits = (uint64_t)value;
		writeU32(p, (uint32_t)(bits >> 32));
		writeU32(p + 4, (uint32_t)bits);
	}

	uint32_t allocateId() {
		uint32_t id = nextId_++;
		if (nextId_ == 0) nextId_ = 1;
		return id;
	}

	size_t makeEnvelope(uint8_t operation, const uint8_t *payload, size_t payloadLength,
			uint32_t id, uint8_t *envelope) {
		size_t length = 16 + payloadLength;
		if (length > PM_PROTOCOL_MAX_REQUEST) return 0;
		envelope[0] = PM_PROTOCOL_VERSION;
		envelope[1] = operation;
		envelope[2] = (uint8_t)(axis_ >> 8);
		envelope[3] = (uint8_t)axis_;
		writeU32(envelope + 4, session_);
		writeU32(envelope + 8, operation == PM_OP_CLAIM ? 0 : generation_);
		writeU32(envelope + 12, id);
		if (payloadLength) memcpy(envelope + 16, payload, payloadLength);
		return length;
	}

	bool transact(uint8_t operation, const uint8_t *payload, size_t payloadLength,
			bool claimRequest) {
		uint8_t envelope[PM_PROTOCOL_MAX_REQUEST];
		size_t length = makeEnvelope(operation, payload, payloadLength, allocateId(), envelope);
		return length && transactEnvelope(envelope, length, claimRequest);
	}

	bool transactEnvelope(const uint8_t *envelope, size_t envelopeLength, bool claimRequest) {
		if (!envelope || envelopeLength < 16 || envelopeLength > PM_PROTOCOL_MAX_REQUEST) return false;
		uint8_t packetPayload[PM_TRANSPORT_MAX_PAYLOAD];
		packetPayload[0] = PM_COMM_PACKET_ID;
		memcpy(packetPayload + 1, envelope, envelopeLength);
		size_t packetLength = envelopeLength + 1;
		uint8_t frame[PM_TRANSPORT_MAX_PAYLOAD + 5];
		frame[0] = 2;
		frame[1] = (uint8_t)packetLength;
		memcpy(frame + 2, packetPayload, packetLength);
		uint16_t crc = crc16(packetPayload, packetLength);
		frame[packetLength + 2] = (uint8_t)(crc >> 8);
		frame[packetLength + 3] = (uint8_t)crc;
		frame[packetLength + 4] = 3;
		stream_.write(frame, packetLength + 5);
		stream_.flush();

		uint8_t received[PM_TRANSPORT_MAX_PAYLOAD];
		size_t receivedLength = 0;
		if (!readFrame(received, receivedLength, 1000) || receivedLength < 1 ||
				received[0] != PM_COMM_PACKET_ID) return false;
		responseLength_ = receivedLength - 1;
		if (responseLength_ > sizeof(response_)) return false;
		memcpy(response_, received + 1, responseLength_);
		if (responseLength_ < 24 || response_[0] != PM_PROTOCOL_VERSION ||
				response_[1] != (uint8_t)(envelope[1] | 0x80) ||
				response_[2] != envelope[2] || response_[3] != envelope[3] ||
				readU32(response_ + 4) != readU32(envelope + 4) ||
				readU32(response_ + 12) != readU32(envelope + 12) ||
				(!claimRequest && readU32(response_ + 8) != readU32(envelope + 8))) return false;
		lastResult_ = response_[16];
		return true;
	}

	bool readByte(uint8_t &value, uint32_t started, uint32_t timeoutMs) {
		while ((uint32_t)(millis() - started) < timeoutMs) {
			if (stream_.available() > 0) {
				int byte = stream_.read();
				if (byte >= 0) {
					value = (uint8_t)byte;
					return true;
				}
			}
		}
		return false;
	}

	bool readFrame(uint8_t *payload, size_t &payloadLength, uint32_t timeoutMs) {
		uint32_t started = millis();
		uint8_t value;
		do {
			if (!readByte(value, started, timeoutMs)) return false;
		} while (value != 2);
		if (!readByte(value, started, timeoutMs)) return false;
		size_t length = value;
		if (length == 0 || length > PM_TRANSPORT_MAX_PAYLOAD) return false;
		uint8_t remainder[PM_TRANSPORT_MAX_PAYLOAD + 3];
		for (size_t i = 0; i < length + 3; ++i) {
			if (!readByte(remainder[i], started, timeoutMs)) return false;
		}
		uint16_t receivedCrc = (uint16_t)remainder[length] << 8 | remainder[length + 1];
		if (remainder[length + 2] != 3 || crc16(remainder, length) != receivedCrc) return false;
		memcpy(payload, remainder, length);
		payloadLength = length;
		return true;
	}
};

#endif
