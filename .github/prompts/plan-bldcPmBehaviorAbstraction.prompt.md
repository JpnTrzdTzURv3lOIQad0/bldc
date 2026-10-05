# BLDC PM Behavior Abstraction

## Summary

Build a capability-oriented motion layer on the existing BLDC PM core, using the 22 sketches under `C:/external.source/vesc/bldc/.github/aiworkspace/clearpath/**` as behavioral references.

The target is native encoder-based FOC motion with a versioned USB serial API. Every example must map to tested software behavior or an explicit unsupported capability. Complete the core corrections below before enabling firmware motion.

## Implementation

### Phase 0: Capability and safety contract

1. Extend the existing ClearPath behavior map to cover all 22 sketches: 20 mode/HLFB examples, StepAndDirection, and FollowEncoder. Record inputs, units, configuration dependencies, completion/status semantics, faults, and hardware requirements. Link the local sketches as the reference sources.

2. Define signed 64-bit axis position counts, velocity/acceleration units, transmission scaling, polarity, sensor identity, acquisition freshness, commutation validity, reference/reset semantics, ownership, bounds, and terminal results.

3. Confirm board/revision, encoder, mechanics, travel, electrical limits, regeneration constraints, stop response, and required accuracy before powered commissioning. Unqualified hardware capabilities remain disabled.

### Phase 1: Core prerequisites and abstraction

4. Retain the responsibilities of `pm_interface`, `pm_trajectory`, `pm_feedback`, and `pm_control`, but explicitly extend their contracts. Preserve legacy motor APIs and configuration schemas.

5. **Separate fault latching from ownership and enable state.** Revocation, disable, abort, and reconnect must preserve latched faults. Enable and motion remain rejected until explicit clear-fault succeeds using firmware-derived fault and feedback health plus caller acknowledgement. Clear-fault leaves the axis disabled and does not restore cancelled work.

6. **Define concurrency and output publication.** The outer-control scheduler owns normal lifecycle, trajectory, and controller updates. Other contexts publish bounded requests through synchronized mailboxes. Asynchronous fault/revoke paths atomically invalidate ownership and output authorization. Before publishing current demand, recheck generation, authorization, fault state, and deadline under the same synchronization contract. Publish coherent status snapshots; do not rely on unsynchronized struct copies or `volatile`.

7. **Give stopping priority over pending motion.** Provide a separate bounded stop/abort path that cannot return BUSY merely because the motion mailbox is occupied. Abort/disable override decelerated stop; fault/revocation override all normal requests. Cancel pending motion and prevent later queued work from restarting the axis.

8. **Preserve integer position precision through trajectories.** Keep absolute origins and endpoints as signed 64-bit counts; perform profile calculations in bounded coordinates relative to an integer origin. Never convert full absolute positions to float. Use checked integer arithmetic, retain fractional progress, and reject profiles outside the supported numerical accuracy envelope. Round intermediate samples consistently and publish the exact integer endpoint at profile completion. Apply the same representation to retargeting and stopping.

9. **Make acquisition timing compatible with the scheduler.** Extend feedback acquisition timestamps and age/gap calculations to a wrap-safe microsecond timebase. Advance sample sequence only on an actual acquisition. Repeated cached reads must not refresh freshness. Validate sensor update rate, scheduler rate, maximum velocity, and modulo-unwrapping limits together; reject unsupported combinations.

10. **Add explicit control modes.** Support position control, velocity control, bounded current demand, and restricted homing velocity control. Homing may operate without an axis reference but must require fresh, healthy feedback and valid commutation. Normal position commands remain reference-gated. Do not fabricate a reference to bypass controller checks.

11. **Support continuous setpoint updates.** Add velocity ramping and replacement of active velocity/follow/current targets. Same-mode updates preserve ramp continuity and do not disable output. Reaching target velocity is a nonterminal `at_target_velocity` status qualified by fresh measured velocity; the mode remains active. A zero velocity target ramps to zero and remains in velocity mode. Explicit stop completes into position hold; abort releases output. Cross-mode requests require stop completion first.

12. Add host-testable `pm_behavior` and `pm_input` layers. Keep transport and physical I/O outside behavior logic. Use fixed bounded storage and explicit unsupported results. Normalize timestamped analog, digital, selector, trigger, quadrature, index, home, and limit inputs with filtering/debounce, range validation, polarity, freshness, and ownership.

### Phase 2: Native firmware runtime

13. Integrate PM into the existing FOC outer-control scheduler after Phase 1 tests pass. Use qualified encoder snapshots, measured bounded elapsed time, per-axis ownership, and expiring current demand. The current-consumption path must reject expired or invalidated PM output even if the outer scheduler stops running. Preserve native current, voltage, thermal, and electrical protections.

14. Revoke PM at legacy takeover, timeout/lease loss, application stop, fault/kill, encoder reset, configuration change, flash save, and shutdown. Keep explicit arbitration between legacy and PM output. PM status traffic must not renew motion leases or mask the existing host timeout or independent watchdog.

15. Keep PM disabled and unreferenced at boot behind an opt-in build/configuration gate. Compile and validate the disabled path before enabling any hardware capability.

### Phase 3: Motion behavior families

16. Map SD position and StepAndDirection intent to absolute/relative profiles, retargeting, controlled stop, and abort. Relative requests use the accepted commanded endpoint. Positional completion requires profile completion plus fresh measured position and velocity satisfying a continuous settle window.

17. Map FollowEncoder to position-follow and velocity-follow with independent command-source identity, scaling, inversion, filtering, slew bounds, index diagnostics, and lock/freeze policy. On command-source loss, stop under valid motor feedback; loss of required motor feedback invalidates output. Recovery requires an explicit new request.

18. Implement switch homing with initially-active handling, backoff, seek, controlled stop, slow re-approach, reference assignment, optional offset, and timeout/travel/current bounds. Use the restricted unreferenced control mode. At reference assignment, atomically update feedback offset, lifecycle reference, trajectory origin, and commanded endpoint. Complete HOME only after any offset move settles. Failed homing leaves the reference invalid. Hard-stop homing remains unsupported until separately qualified.

19. Map MC position tables, increment selections/repetitions, and pulse-burst distances to validated software parameters/events. Include pulse-burst alternate-speed selection. Preserve event counts without reproducing enable-pin pulse signaling.

20. Map selected/ramped velocity, ManualVelocity deltas, digital position/velocity/torque, and variable current limits to the explicit control modes. Current/torque operation requires independent overspeed/travel protection. Report current and only calibrated, qualified estimated torque.

21. Map status-register and HLFB examples to lifecycle, active command/result, targets, measured position/velocity, at-speed status, limits, feedback age/health, current, saturation, and faults. Physical HLFB output is not implied.

22. Enable local dual-axis common-start only with two independently qualified feedback sources and output stages. Require atomic group acceptance, a shared start epoch, both-axis completion, and common stop/fault policy. Contouring and distributed synchronization remain excluded.

## USB API and documentation

- Append a dedicated packet identifier with a versioned PM envelope. Include capabilities/configuration, owner lease, enable/disable, reference/home, absolute/relative motion, continuous-mode selection/setpoint updates, stop/abort, clear-fault, and status.
- Reuse packet framing and USB dispatch. Preserve legacy `COMM_SET_POS` and existing custom-data/config callback slots. CAN transport remains deferred.
- Validate length, version, axis, session/generation, command ID, lease, and numeric bounds before dispatch. Make retries idempotent, including relative moves and incremental events.
- Distinguish request acceptance, continuous at-target status, and terminal results. Bound response storage and rates; return explicit capability rejection reasons.
- Document the behavior matrix, updated core contracts, protocol, limitations, and commissioning procedure in `C:/external.source/vesc/bldc/documentation/pm_behavior_abstraction.md`.

## Verification and rollout

Extend the host suite with deterministic fake input, feedback, clock, and output backends. Required acceptance cases include:

- Every example’s supported mapping or explicit unsupported result.
- Fault → revoke/disable → reclaim → enable remaining rejected until valid clear-fault.
- Revocation between computation and output publication, coherent status reads, and stop/abort with a full motion mailbox.
- Boot → unreferenced homing → reference assignment → relative move through the combined core; homing failure and initially-active switch handling.
- Single-count moves, retargeting, stopping, and overflow rejection at large positive/negative absolute positions, including beyond `2^24`.
- Multiple acquisitions within one millisecond, timestamp wrap, duplicate samples, frozen producers, sensor reset, and unsupported sampling/speed combinations.
- Repeated continuous updates, reversals, zero-speed ramps, nonterminal at-speed reporting, source loss, and explicit mode transitions.
- Bounds, settling, lease expiry, stale ownership, cancellation, recovery, and no unexpected restart.
- USB truncation, oversized payloads, invalid versions/axes, stale generations, duplicate/replayed IDs, and status responses.

Run the PM host suite and `all_ut_run` using the repository’s GoogleTest configuration. Build confirmed single-axis hardware with PM disabled/enabled and a supported dual-motor target where applicable. Check warnings, size, scheduler/ISR timing, and legacy regression behavior.

Perform HIL only after hardware and safety limits are recorded: output-disabled encoder observation, low-energy polarity checks, bounded moves, homing/limits, communications loss, sensor loss/reset, scheduler overrun, and thermal/regeneration behavior. Measure load position independently before claiming accuracy. Preserve an opt-in rollout and rollback path.

## Assumptions and defaults

- ClearPath/ClearCore sketches are behavioral references; no ClearCore library or wrapper changes are included.
- STEP/DIR electrical output, MC A/B signaling, HLFB electrical output, MSP compatibility, and industrial safety equivalence are excluded.
- Flipsky 75 is only a candidate; hardware selection remains a commissioning prerequisite.
- Existing PM modules are partial building blocks. Their isolated tests do not establish integrated behavior, physical accuracy, or safety.
- Unqualified adapters, hard-stop homing, and dual-axis operation report unsupported until their prerequisites are satisfied.
