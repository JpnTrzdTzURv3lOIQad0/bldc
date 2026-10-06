# PM behavior abstraction

PM is an opt-in, single-axis, encoder-based FOC motion subsystem. ClearPath
examples describe motion intent; this implementation does not generate STEP/DIR,
MC A/B or HLFB waveforms, implement MSP configuration, or claim industrial safety.
See [the 22-example matrix](clearpath_examples_behavior_map.md).

## Build and hardware gate

`PM_INTERFACE_ENABLE=0` is the default. `PM_INTERFACE_ENABLE=1` includes the pure
core, runtime and USB handler. No shipped board is qualified: the weak
`pm_board_configure` and `pm_board_sample` hooks return false. Discovery and
status remain available; motion returns `UNSUPPORTED`. This is intentional,
not a sensorless fallback. Neither Flipsky 75 nor another board is implicitly
selected for commissioning. Dual-axis commands and hard-stop homing are unsupported.

A board adapter must provide an immutable validated `pm_runtime_config_t` and
actual acquisition snapshots. It must qualify GPIO/timer/ADC ownership, independent
command encoder resources, commutation calibration, polarity, transmission,
physical limits, sample rate, stopping/regeneration energy, and scheduler budget.
Cached angle reads must never create new acquisition timestamps or sequences.
The board sample contains home validity, directional limit levels, backend fault
health and measured current. Input conditioning must precede that snapshot.
Only a qualified adapter may return true from configuration. Current output uses
the native FOC sign convention; the adapter must verify this agrees with the
canonical axis direction before qualification. Homing is not commutation calibration.

The native PID thread invokes PM; its enabled stack allocation is 4096 bytes.
The current ISR consumes expiring demand independently of that thread. PM demand
enters the existing FOC current path before its native current/voltage limits.
Legacy setters, faults, host timeout/kill, configuration changes, encoder reset,
application stop, flash saves and shutdown revoke PM. PM does not call
`timeout_reset`: applications must keep the legacy host watchdog alive separately
from the PM lease. A read-only request renews neither watchdog nor lease.

## Units, bounds and lifecycle

- Position: signed 64-bit canonical axis counts. Sensor modulo counts, inversion,
  source ID, counts/revolution and reset epoch belong to `pm_feedback`.
- Velocity: counts/second; acceleration/deceleration: counts/second squared.
  Current command: signed milliamps on the wire; controller and telemetry
  calculations use amps. Torque estimation is not calibrated or advertised.
- Feedback acquisition, freshness, source timeout, output age and lease use
  unsigned 32-bit microseconds with wrap-safe differences and intervals below
  `2^31`. Sensor sequence advances only on acquisition. Configurations permitting
  half a revolution between samples are rejected. Reset or lost feedback drops
  the reference. Settling uses a separately accumulated millisecond clock.
- Absolute positions are never converted to float. `pm_profile` retains an
  integer origin and endpoint and uses a bounded local trajectory. Local motion,
  including braking excursions, is limited to 65536 counts. Requests outside
  that envelope are rejected; implicit segmentation is not provided. Samples
  round to nearest count, ties away from zero, and final samples use the exact
  integer endpoint. Phase evaluation retains elapsed fractional time and avoids
  repeated position-integration rounding. The software resolution envelope is
  one count, not a physical accuracy claim.
- Relative motion uses the accepted commanded endpoint. Position completion
  requires profile completion and a continuous fresh measured settle window.
  Same-mode velocity/current updates supersede commands without disabling
  output. At-speed is nonterminal. Zero velocity remains an active velocity
  command. Cross-mode transitions require an explicit stop first.
- A completed decelerated stop holds its measured-motion stopping endpoint.
  Abort/disable release output. Switch homing alone may drive while unreferenced,
  with fresh healthy commutated feedback. It backs off an initially active switch,
  seeks when necessary, brakes, backs off, brakes, approaches slowly, brakes,
  assigns the reference and moves to the configured offset coordinate. The
  offset is an absolute axis coordinate, not a second reference assignment.
  Home/travel/current/settle failures invalidate the homing reference.
- Faults are independent of enable and ownership. Disable, revoke, reconnect
  and abort cannot clear them. Clear-fault requires caller acknowledgement plus
  current backend health and leaves output disabled. Fault code zero is reserved.

## Synchronization, inputs and replay

`pm_interface`, profile, control, feedback, input and behavior functions are pure
single-context components. Firmware and transports use `pm_runtime` rather than
mutating them directly. Runtime APIs require lock/unlock callbacks that serialize
thread and ISR access. Initialization occurs before publishing the runtime.
There is exactly one tick caller. It copies a bounded engine snapshot under the
lock, computes outside the lock, and commits only if revision and authorization
epoch still match. Revocation invalidates output under that same lock. The ISR
checks both lease and output deadline on every PM current consumption. A stalled
outer scheduler cannot keep its last current demand alive.

There is one normal mailbox and a separate priority slot. Stop cancels queued
motion. Abort/disable override stop; lower-priority stop cannot displace a pending
release. Fault/revoke discard both. Status snapshots are copied under the lock.
No heap allocation is used. Critical-section and ISR worst-case timings still
require measurement on the selected hardware.

`pm_input` validates source identity, owner session/generation, sequence, range,
health and acquisition age before applying polarity and an optional positive
rational scale. Both scale fields zero mean identity; scaling truncates toward
zero and rejects intermediate overflow. Digital debounce requires timestamped
observations spanning the configured interval. Trigger counts are not debounced
or coalesced; each value is an explicit repetition count. Board adapters may
filter analog noise before normalization; motion profiles/velocity ramps bound
the resulting demand. Source snapshots and adapter settings must be serialized
by their producer before being passed to behavior dispatch.

Follow uses a source distinct from motor feedback. Position freeze retains the
accepted target; velocity freeze ramps to zero. Valid frozen observations still
refresh source acquisition age. Source loss ends follow and decelerates using
healthy motor feedback. Recovery requires an explicit new command. Motor feedback
loss instead invalidates output immediately.

Command IDs are nonzero, strictly increasing within an owner generation; wrap
requires a new owner generation. Eight accepted requests are retained for exact
retry matching. Older evicted IDs are rejected, never re-executed. Reusing an ID
with different content is invalid. BUSY requests can be retried. The USB behavior
cache compares the original bytes before remapping ManualVelocity deltas or
increment counts. Owner generation is checked before replay lookup. Claim retries
return the existing generation without extending its lease. Leases require
explicit renewal. A reboot requires a new host session; sessions are not persistent.
Status exposes the latest terminal result, not an unbounded result history.

## VESC Packet v1

Packet ID `COMM_PM = 160` is appended to the existing enumeration and handled by
the shared command dispatcher over USB CDC, configured UART packet links, and
VESC CAN packet forwarding. Existing packet framing supplies framing/CRC; PM
replies use the request's normal reply callback. All multibyte integers are
big-endian; signed values use two's complement. Lengths must match exactly.
Maximum request payload is 36 bytes after COMM_PM; maximum response buffer is
128 bytes. Replies are synchronous and bounded to one per request; there is no
unsolicited telemetry stream.

Every request begins with this 16-byte header:

| Offset | Field |
| --- | --- |
| 0 | Version, 1 byte, must equal 1 |
| 1 | Operation, 1 byte |
| 2 | Axis, unsigned 16 bits; firmware currently supports axis 0 |
| 4 | Owner session, unsigned 32 bits |
| 8 | Owner generation, unsigned 32 bits |
| 12 | Command ID, unsigned 32 bits |

| Operation | Value | Total bytes | Payload after header |
| --- | --- | --- | --- |
| Capabilities | 0 | 16 | None |
| Configuration | 1 | 16 | None; read-only commissioned limits |
| Claim | 2 | 20 | Lease duration u32 microseconds; generation must be zero |
| Renew | 3 | 20 | Lease duration u32 microseconds |
| Command | 4 | 34 | Type u8, flags u8, value i64, speed limit u32, current limit u32 mA |
| Status | 5 | 16 | None |
| Behavior | 6 | 36 | Type u8, selection u8, flags u8, reserved zero u8, repetitions u32, value i64, current limit u32 mA |

Command types, in order from zero: enable, disable, set-reference, home,
move-absolute, move-relative, velocity, stop-decelerated, abort-release,
clear-fault, current. Flag bit 0 permits replacement; bit 1 acknowledges
clear-fault. Other bits are rejected. A zero speed/current limit uses the
commissioned maximum; a nonzero value can only lower it. The value field is
counts for position/reference, counts/second for velocity, and mA for current.
Health flags are never accepted from the wire.

Behavior types, in order from zero: absolute, relative, velocity, current,
preset, increment, selected-velocity, manual-velocity, follow-position,
follow-velocity, pulse-burst. Selections are zero-based. Behavior flag bit 0
selects alternate pulse-burst speed. Follow requires a qualified local input
and cannot be supplied by inventing encoder samples in USB requests. Preset
tables and adapter settings are commissioned configuration; v1 USB does not
persist or rewrite them.

All replies echo the header with version 1 and operation bit 7 set. Claim
replies return the assigned generation. Bytes 16–23 contain result u8,
unsupported reason u8, lifecycle u8, latest terminal result u8, latest terminal
command ID u32. Result values are the `pm_result_t` enumeration: accepted/pending,
busy, invalid, stale-owner, not-ready, unsupported. Acceptance is never proof of
motion completion. Poll status to observe lifecycle and terminal result.

Capabilities/configuration append capability bits at 24, maximum lease at 28,
maximum output age at 32, minimum/maximum position i64 at 36/44, maximum velocity
u32 at 52, maximum current mA u32 at 56, then 22 example reason bytes at 60.
Capability bits are position, velocity, current, switch home, follow, presets,
telemetry, torque estimate (bits 0–7). Unsupported reasons are none, hardware,
hard-stop, dual-axis, torque calibration, capability (values 0–5).

Status appends active command ID at 24, fault at 28, measured position i64 at 32,
commanded endpoint i64 at 40, feedback age us at 48; bytes 52–59 are enabled,
referenced, at-speed, feedback-valid, current-saturated, velocity-saturated,
output-valid, commutation-valid. Measured velocity i64 counts/second is at 60;
current demand i32 mA is at 68. These are diagnostics, not HLFB electrical output.

## Verification and commissioning

Run `make GTEST_DIR=<googletest-directory> ut_pm_motion_run` and `all_ut_run`.
The PM suite includes fake feedback/clock/input and revocation interleaving,
fault-latch recovery, full-mailbox stop, unreferenced homing, large coordinates,
sub-millisecond acquisition and wrap, continuous updates, input loss, and USB
malformed/replayed requests. Firmware builds must use separate PM-off/PM-on
output directories to avoid reusing objects compiled with a different gate.

Before powered commissioning, record board and PCB revision, encoder interface
and counts/revolution, commutation calibration and polarity, motor/load ratio,
travel and independent limits, current/voltage/thermal/regeneration limits,
stopping response, input resource ownership, accuracy target, and emergency stop.
Measure snapshot, scheduler, lock, stack and ISR budgets at the intended rate.
Then perform output-disabled encoder observation, low-energy polarity tests,
bounded moves, switch homing, directional limits, communication/lease loss,
sensor unplug/reset, scheduler stall and thermal/regeneration tests. Independent
load measurement is required for physical accuracy claims. No hardware has been
flashed or commissioned by the host implementation.
