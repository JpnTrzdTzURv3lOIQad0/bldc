#include "pm_protocol.h"
#include <string.h>

static uint32_t u32(const uint8_t *b) {
  return (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 |
         b[3];
}
static int64_t i64(const uint8_t *b) {
  uint64_t n = (uint64_t)u32(b) << 32 | u32(b + 4);
  /* Avoid implementation-defined unsigned-to-signed conversion. */
  return n <= INT64_MAX ? (int64_t)n : -1 - (int64_t)(UINT64_MAX - n);
}
static void put32(uint8_t *b, uint32_t v) {
  b[0] = (uint8_t)(v >> 24);
  b[1] = (uint8_t)(v >> 16);
  b[2] = (uint8_t)(v >> 8);
  b[3] = (uint8_t)v;
}
static void put64(uint8_t *b, int64_t v) {
  put32(b, (uint32_t)((uint64_t)v >> 32));
  put32(b + 4, (uint32_t)v);
}
void pm_protocol_init(pm_protocol_t *p, pm_runtime_t *r) {
  memset(p, 0, sizeof(*p));
  p->runtime = r;
}
size_t pm_protocol_process(pm_protocol_t *p, const uint8_t *b, size_t n,
                           uint8_t *out, size_t capacity, uint32_t now) {
  if (!p || !p->runtime || !b || !out || capacity < PM_PROTOCOL_MAX_RESPONSE)
    return 0;
  memset(out, 0, PM_PROTOCOL_MAX_RESPONSE);
  out[0] = PM_PROTOCOL_VERSION;
  pm_runtime_t *r = p->runtime;
  float ignored;
  pm_runtime_consume(r, now,
                     &ignored); /* Enforce expiry; never renew it on reads. */
  pm_runtime_status_t status;
  pm_runtime_status(r, &status);
  pm_result_t result = PM_RESULT_INVALID;
  uint8_t op = n > 1 ? b[1] : 255;
  out[1] = op | 0x80U;
  uint32_t session = 0, generation = 0, id = 0;
  uint16_t axis = 0;
  if (n >= PM_PROTOCOL_HEADER) {
    axis = (uint16_t)((uint16_t)b[2] << 8 | b[3]);
    session = u32(b + 4);
    generation = u32(b + 8);
    id = u32(b + 12);
    memcpy(out + 2, b + 2, 14);
  }
  bool valid = n >= PM_PROTOCOL_HEADER && n <= PM_PROTOCOL_MAX_REQUEST &&
               b[0] == PM_PROTOCOL_VERSION && axis == r->config.axis_id;
  bool command = op == PM_USB_COMMAND || op == PM_USB_BEHAVIOR;
  if (valid && command &&
      (session == 0 || session != status.axis.owner_session ||
       generation != status.axis.owner_generation)) {
    result = PM_RESULT_STALE_OWNER;
    valid = false;
  }
  bool replay = false;
  if (valid && command) {
    for (unsigned i = 0; i < PM_REPLAY_SLOTS; ++i) {
      pm_protocol_replay_t *slot = &p->replay[i];
      if (slot->length >= PM_PROTOCOL_HEADER &&
          u32(slot->data + 4) == session && u32(slot->data + 8) == generation &&
          u32(slot->data + 12) == id) {
        result = slot->length == n && !memcmp(slot->data, b, n)
                     ? slot->result
                     : PM_RESULT_INVALID;
        replay = true;
        break;
      }
    }
  }
  if (valid && !replay) {
    if (op == PM_USB_CAPABILITIES || op == PM_USB_CONFIG ||
        op == PM_USB_STATUS) {
      if (n == PM_PROTOCOL_HEADER)
        result = PM_RESULT_ACCEPTED_PENDING;
    } else if (op == PM_USB_CLAIM && n == 20 && generation == 0) {
      result = pm_runtime_claim(r, session, u32(b + 16), now, &generation);
      put32(out + 8, generation);
    } else if (op == PM_USB_RENEW && n == 20) {
      result = pm_runtime_renew(r, session, generation, u32(b + 16), now);
    } else if (op == PM_USB_COMMAND && n == 34 && (b[17] & ~3U) == 0) {
      pm_command_t cmd = {0};
      cmd.axis_id = axis;
      cmd.owner_session = session;
      cmd.owner_generation = generation;
      cmd.command_id = id;
      cmd.type = (pm_command_type_t)b[16];
      cmd.replace_active = b[17] & 1U;
      cmd.acknowledged = b[17] & 2U;
      cmd.value_ticks = i64(b + 18);
      cmd.speed_limit = (float)u32(b + 26);
      cmd.current_limit_amps = (float)u32(b + 30) / 1000.0f;
      result = pm_runtime_submit(r, &cmd, now);
    } else if (op == PM_USB_BEHAVIOR && n == 36 && (b[18] & ~1U) == 0 &&
               b[19] == 0) {
      pm_behavior_request_t req = {0};
      pm_command_t cmd = {0};
      req.type = (pm_behavior_type_t)b[16];
      req.selection = b[17];
      req.alternate_speed = b[18];
      req.repetitions = u32(b + 20);
      req.value = i64(b + 24);
      cmd.axis_id = axis;
      cmd.owner_session = session;
      cmd.owner_generation = generation;
      cmd.command_id = id;
      cmd.current_limit_amps = (float)u32(b + 32) / 1000.0f;
      result = pm_runtime_behavior(r, &req, NULL, &cmd, now);
    } else if (op > PM_USB_BEHAVIOR)
      result = PM_RESULT_UNSUPPORTED;
    if (command && result == PM_RESULT_ACCEPTED_PENDING) {
      pm_protocol_replay_t *slot = &p->replay[p->next];
      memcpy(slot->data, b, n);
      slot->length = (uint8_t)n;
      slot->result = result;
      p->next = (p->next + 1) % PM_REPLAY_SLOTS;
    }
  }
  pm_runtime_status(r, &status);
  out[16] = (uint8_t)result;
  out[17] = result == PM_RESULT_UNSUPPORTED ? PM_UNSUPPORTED_HARDWARE
                                            : PM_UNSUPPORTED_NONE;
  out[18] = (uint8_t)status.axis.lifecycle;
  out[19] = (uint8_t)status.axis.last_terminal_result;
  put32(out + 20, status.axis.last_terminal_command_id);
  size_t size = 24;
  if (valid && result == PM_RESULT_ACCEPTED_PENDING &&
      (op == PM_USB_CAPABILITIES || op == PM_USB_CONFIG)) {
    put32(out + 24, status.capabilities);
    put32(out + 28, r->config.lease_max_us);
    put32(out + 32, r->config.output_max_age_us);
    put64(out + 36, r->config.minimum_position);
    put64(out + 44, r->config.maximum_position);
    put32(out + 52, (uint32_t)r->config.control.max_velocity_counts_per_second);
    put32(out + 56, (uint32_t)(r->config.control.current_limit_amps * 1000.0f));
    for (unsigned i = 0; i < PM_EX_COUNT; ++i)
      out[60 + i] =
          (uint8_t)pm_behavior_example((pm_example_t)i, status.capabilities);
    size = 60 + PM_EX_COUNT;
  } else if (valid && result == PM_RESULT_ACCEPTED_PENDING &&
             op == PM_USB_STATUS) {
    put32(out + 24, status.axis.active_command_id);
    put32(out + 28, status.axis.fault_code);
    put64(out + 32, status.feedback.axis_position_counts);
    put64(out + 40, status.axis.last_commanded_endpoint_ticks);
    uint32_t age = now - status.feedback.acquisition_time_us;
    put32(out + 48, age);
    out[52] = status.axis.enabled;
    out[53] = status.axis.referenced;
    out[54] = status.axis.at_target_velocity;
    out[55] =
        status.feedback.valid && age <= r->config.feedback.max_sample_age_us;
    out[56] = status.control.current_saturated;
    out[57] = status.control.velocity_saturated;
    out[58] = status.output_valid;
    out[59] = status.feedback.commutation_valid;
    put64(out + 60, (int64_t)status.feedback.velocity_counts_per_second);
    put32(out + 68,
          (uint32_t)(int32_t)(status.control.iq_demand_amps * 1000.0f));
    size = 72;
  }
  return size;
}
