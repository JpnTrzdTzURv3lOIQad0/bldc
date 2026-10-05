#ifndef PM_PROTOCOL_H
#define PM_PROTOCOL_H
#include "pm_runtime.h"
#define PM_PROTOCOL_VERSION 1U
#define PM_PROTOCOL_HEADER 16U
#define PM_PROTOCOL_MAX_REQUEST 36U
#define PM_PROTOCOL_MAX_RESPONSE 128U
typedef enum { PM_USB_CAPABILITIES, PM_USB_CONFIG, PM_USB_CLAIM,
	PM_USB_RENEW, PM_USB_COMMAND, PM_USB_STATUS, PM_USB_BEHAVIOR } pm_usb_operation_t;
typedef struct {
	uint8_t data[PM_PROTOCOL_MAX_REQUEST];
	uint8_t length;
	pm_result_t result;
} pm_protocol_replay_t;
typedef struct {
	pm_runtime_t *runtime;
	pm_protocol_replay_t replay[PM_REPLAY_SLOTS];
	unsigned next;
} pm_protocol_t;
void pm_protocol_init(pm_protocol_t *p, pm_runtime_t *runtime);
/* Payload after the dedicated COMM_PM packet ID. One transport thread owns p.
 * All integers are big-endian. No floats or C structs are sent on the wire. */
size_t pm_protocol_process(pm_protocol_t *p, const uint8_t *request, size_t length,
		uint8_t *response, size_t capacity, uint32_t now_us);
#endif
