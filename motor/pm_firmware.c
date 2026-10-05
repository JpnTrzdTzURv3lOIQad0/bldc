#include "pm_firmware.h"
#include "pm_protocol.h"
#include "ch.h"
#include "timer.h"
#include "timeout.h"
#include <string.h>

static pm_runtime_t runtime;
static pm_protocol_t protocol;
static bool initialized;
static uint32_t timer_previous, timer_fraction, clock_us;
static uintptr_t lock(void *unused) { (void)unused; return (uintptr_t)chSysGetStatusAndLockX(); }
static void unlock(void *unused, uintptr_t state) { (void)unused; chSysRestoreStatusX((syssts_t)state); }
uint32_t pm_firmware_now_us(void) {
	uintptr_t state = lock(NULL);
	uint32_t ticks = timer_time_now();
	uint32_t delta = ticks - timer_previous;
	timer_previous = ticks;
	/* TIM5 is 14 MHz. Accumulate differences before division so its 32-bit
	 * wrap does not shorten the microsecond clock's wrap period. */
	clock_us += delta / 14U;
	timer_fraction += delta % 14U;
	clock_us += timer_fraction / 14U; timer_fraction %= 14U;
	uint32_t result = clock_us;
	unlock(NULL, state); return result;
}
__attribute__((weak)) bool pm_board_configure(pm_runtime_config_t *config) { (void)config; return false; }
__attribute__((weak)) bool pm_board_sample(pm_runtime_sample_t *sample, uint32_t now) { (void)sample; (void)now; return false; }
void pm_firmware_init(void) {
	/* These are inert discovery defaults, not a commissioned motor setup. */
	pm_runtime_config_t c = {0};
	c.lease_max_us = 1000000; c.output_max_age_us = 50000; c.source_max_age_us = 100000;
	c.minimum_position = -65536; c.maximum_position = 65536;
	c.acceleration = 1000.0f; c.deceleration = 1000.0f;
	c.feedback.counts_per_revolution = 4096; c.feedback.max_counts_per_second = 10000;
	c.feedback.max_sample_gap_us = 10000; c.feedback.max_sample_age_us = 10000;
	c.control.max_velocity_counts_per_second = 1000.0f;
	c.control.current_limit_amps = 1.0f; c.control.max_dt_seconds = 0.05f;
	c.control.following_error_limit_counts = 100;
	c.settle.position_tolerance_counts = 1; c.settle.velocity_tolerance_counts_per_second = 1.0f;
	c.settle.settle_duration_ms = 20; c.settle.timeout_ms = 1000;
	c.qualified = pm_board_configure(&c);
	initialized = pm_runtime_init(&runtime, &c, lock, unlock, NULL);
	if (initialized) pm_protocol_init(&protocol, &runtime);
	timer_previous = timer_time_now();
}
void pm_firmware_tick(void) {
	if (!initialized) return;
	uint32_t now = pm_firmware_now_us();
	pm_runtime_sample_t sample = {0};
	if (!pm_board_sample(&sample, now)) sample.fault_clear = false;
	if (timeout_has_timeout() || timeout_kill_sw_active()) sample.fault_clear = false;
	pm_runtime_tick(&runtime, &sample, now);
}
void pm_firmware_revoke(unsigned axis, uint32_t fault) {
	if (initialized && axis == 0) pm_runtime_revoke(&runtime, fault);
}
void pm_firmware_revoke_all(void) { pm_firmware_revoke(0, 0); }
bool pm_firmware_current(unsigned axis, float *current) {
	return initialized && axis == 0 && pm_runtime_consume(&runtime, pm_firmware_now_us(), current);
}
size_t pm_firmware_packet(const uint8_t *request, size_t length, uint8_t *response, size_t capacity) {
	if (!initialized) return 0;
	return pm_protocol_process(&protocol, request, length, response, capacity, pm_firmware_now_us());
}
