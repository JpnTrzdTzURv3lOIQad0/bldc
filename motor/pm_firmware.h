#ifndef PM_FIRMWARE_H
#define PM_FIRMWARE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifndef PM_INTERFACE_ENABLE
#define PM_INTERFACE_ENABLE 0
#endif
#if PM_INTERFACE_ENABLE
#include "pm_runtime.h"
void pm_firmware_init(void);
void pm_firmware_tick(void);
uint32_t pm_firmware_now_us(void);
void pm_firmware_revoke(unsigned axis, uint32_t fault);
void pm_firmware_revoke_all(void);
bool pm_firmware_current(unsigned axis, float *current);
size_t pm_firmware_packet(const uint8_t *request, size_t length, uint8_t *response, size_t capacity);
/* Board qualification seam. Defaults return false; no encoder angle is
 * relabelled as a fresh acquisition. A board implementation must provide
 * genuine acquisition timestamps/sequence, calibrated polarity and fault I/O.
 * Only axis 0 is supported until independent dual feedback is qualified. */
bool pm_board_configure(pm_runtime_config_t *config);
bool pm_board_sample(pm_runtime_sample_t *sample, uint32_t now_us);
#else
static inline void pm_firmware_init(void) {}
static inline void pm_firmware_tick(void) {}
static inline void pm_firmware_revoke(unsigned axis, uint32_t fault) { (void)axis; (void)fault; }
static inline void pm_firmware_revoke_all(void) {}
#endif
#endif
