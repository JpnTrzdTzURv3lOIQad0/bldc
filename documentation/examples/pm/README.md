# Arduino PM Serial Examples

`pm.ino` is one Arduino sketch with four selectable workflows. The sketch sends
the documented PM v1 envelope inside the VESC serial packet format
(start byte `0x02`, one-byte payload length, payload, CRC-16/CCITT, end byte
`0x03`). The VESC payload starts with `COMM_PM` (`160`). The shared
`PmSerialClient.h` builds and validates these frames, encodes the big-endian PM
fields, polls status, renews the owner lease, and retains the last command bytes
for exact retry after a lost response.

## Transport And Safety

The current firmware dispatches `COMM_PM` on USB CDC only. It does not dispatch
PM over UART or CAN. `Serial` in the sketch must therefore be a bidirectional
USB CDC connection to the VESC, provided by a USB-host-capable Arduino platform
or a bridge that transports raw bytes to that USB CDC endpoint. A TTL UART wire
connected to the VESC is not a supported transport. Keep diagnostic output on a
separate port by defining `PM_DEBUG_PORT` for the target board; do not print
human-readable text on the PM transport stream.

PM firmware is disabled by default, and the current weak board hooks report
motion unsupported until a board adapter and hardware are qualified. Start with
example 1. Do not try powered motion until the board, encoder, commutation,
mechanics, travel limits, electrical limits, regeneration, and stopping response
have been qualified and recorded as required by
[the PM commissioning contract](../../pm_behavior_abstraction.md#verification-and-commissioning).

These sketches are protocol examples, not a safety controller. Select only one
workflow at a time, use axis 0, keep an independent emergency stop available,
and do not treat request acceptance as motion completion. The examples never
clear a fault automatically except example 4, which must be selected and run
manually after the physical fault cause has been resolved. Clearing leaves the
axis disabled and does not resume cancelled work.

## Select An Example

Open `pm.ino` and set `PM_EXAMPLE` near the top:

- `1`: Read capabilities and status without claiming ownership.
- `2`: Claim the axis, enable it, perform configured switch homing, then make a
  100-count relative move and wait for measured settling. Requires qualified
  position and switch-home capabilities. The axis remains enabled in position
  hold afterward; the sketch continues renewing its lease.
- `3`: Require an existing reference, ramp to 100 counts/second, replace the
  target with zero, wait for measured at-target velocity, request controlled
  stop, and disable.
- `4`: Claim an axis with a latched fault, acknowledge clear-fault, verify the
  fault cleared while the axis stays disabled. Only use after resolving the
  fault cause.

The position and velocity values are canonical axis counts and counts/second,
not motor electrical revolutions. Change example values only after checking the
returned configuration bounds and the commissioned transmission scaling. The
example creates a nonzero session ID for demonstration; production hosts should
use a session identifier with a reliable uniqueness source across restarts.

If desired, define `PM_DEBUG_PORT` as a separate Arduino stream such as
`Serial1` to print progress. `Serial` remains reserved for framed PM traffic.

## Protocol Notes

The helper follows `documentation/pm_behavior_abstraction.md` USB v1 layouts:
capabilities/configuration, claim, renew, command, and status. IDs increase within
the session generation. If a command response times out, retry that same command
with `retryLastCommand()` before issuing a different command; the helper preserves
the exact request bytes and ID. Status polling does not renew the lease, so the
motion examples renew it separately at roughly one third of the selected lease
duration. A lease expiry revokes ownership and invalidates PM output.
