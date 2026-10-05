# ClearPath Example Behavior Map

This catalog covers all 22 local Arduino sketches: 20 under `ClearPathModeExamples`, plus `StepAndDirection` and `EncoderInput/FollowEncoder`. A ClearPath motor's MSP configuration is part of each example's behavior: selected operating mode, input format/filter, homing setup, position/velocity/increment selections, limits, and HLFB mode must agree with the example. The application code is generally a signal adapter and status monitor, not a replacement for the motor's configured mode logic.

For the library/runtime architecture behind these programs, see [ClearPath Motion Implementation Map](clearpath_motion_implementation_map.md).

## ClearPath-MC Series

| Example | Command pattern | Completion, homing, or notable dependency |
| --- | --- | --- |
| [Abs2PositionsHomeToSwitch](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-MC%20Series/MoveAbsolutePosition/Abs2PositionsHomeToSwitch/Abs2PositionsHomeToSwitch.ino) | Input A selects one of two MSP-defined absolute positions. Input B mirrors a DI-6 home sensor; a change callback updates the motor input. | ClearPath performs the configured switch homing. The example waits for HLFB or motor fault after enabling and after each selection; it delays to satisfy configured A/B filter time. |
| [Abs4PositionsHomeToHardStop](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-MC%20Series/MoveAbsolutePosition/Abs4PositionsHomeToHardStop/Abs4PositionsHomeToHardStop.ino) | A/B form a two-bit selector for four MSP-defined absolute positions. | ClearPath performs configured hard-stop homing. The example initializes both inputs, waits for HLFB/fault, then changes the selector and waits for move feedback. |
| [Abs16PositionsHomeToHardStop](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-MC%20Series/MoveAbsolutePosition/Abs16PositionsHomeToHardStop/Abs16PositionsHomeToHardStop.ino) | Sends a pulse count on B followed by an A trigger; the count selects one of 16 MSP-defined absolute positions. | Motor-side hard-stop homing and position table are required. The example rejects indices outside 1..16 and waits for HLFB/fault. |
| [FollowDigitalPosition](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-MC%20Series/FollowDigitalPosition/FollowDigitalPosition.ino) | Scales an analog 0-10 V input to a configured position range and sends it as B PWM. A carries a digital command-lock state, updated from DI-6. | ClearPath follows the commanded position according to its mode and MSP range. PWM offers limited command resolution; HLFB is configured for position/measured-torque feedback. |
| [FollowDigitalTorque](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-MC%20Series/FollowDigitalTorque/FollowDigitalTorque.ino) | Sends signed torque as B PWM magnitude plus A direction state. | Magnitude is scaled against the MSP max-torque setting. The code waits beyond the A input filter interval and then waits for HLFB to assert. Speed limits and torque configuration are motor-side requirements. |
| [FollowDigitalVelocity](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-MC%20Series/FollowDigitalVelocity/FollowDigitalVelocity.ino) | Scales an analog voltage into a nonnegative speed magnitude on B PWM; A selects direction. | Requires matching MSP max-speed and input-filter settings. The example checks range and alerts before applying the command. |
| [FollowDigitalVelocityWithVariableTorque](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-MC%20Series/FollowDigitalVelocityWithVariableTorque/FollowDigitalVelocityWithVariableTorque.ino) | B bipolar PWM commands signed velocity; A PWM selects a torque limit between configured alternate and primary limits. | Both input scales, PWM deadband, max speed, and symmetric torque limits must match MSP. The source notes 8-bit command resolution and does not use HLFB for at-speed completion. |
| [ManualVelocity](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-MC%20Series/ManualVelocity/ManualVelocity.ino) | Emits quadrature transitions on A/B. The number and ordering of transitions encode the change from the previous commanded velocity using an MSP velocity resolution. | Requires exact microsecond signal timing and matching velocity resolution. The example tracks its last requested velocity and waits for HLFB at target speed or a fault. |
| [Move2IncrementsHomeToSwitch](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-MC%20Series/MoveIncrementalDistance/Move2IncrementsHomeToSwitch/Move2IncrementsHomeToSwitch.ino) | A selects one of two configured increments; enable trigger pulses request the number of repetitions. B mirrors the home switch input. | ClearPath's configured switch homing/increment values and trigger width are required. The example waits for the input filter, sends the trigger train, then waits for HLFB/fault. |
| [Move4IncrementsHomeToHardStop](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-MC%20Series/MoveIncrementalDistance/Move4IncrementsHomeToHardStop/Move4IncrementsHomeToHardStop.ino) | A/B select one of four configured increments; enable trigger pulses repeat the selected increment. | ClearPath's hard-stop homing, increment values, trigger width, and input filters must match. The example checks motor fault and HLFB around each move. |
| [PulseBurstPositioning](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-MC%20Series/PulseBurstPositioning/PulseBurstPositioning.ino) | Uses ClearCore's step generator to emit relative pulse distances. Host-side velocity/acceleration limits are set to their maximum so the ClearPath MSP profile controls the pulse-burst move. An enable trigger can select the alternate speed. | Requires matching pulse input format, position resolution, motor-side profile and trigger-pulse setup. Move helper waits for both pulse generation to finish and HLFB assertion, or exits on alert. |
| [RampUpDownToSelectedVelocity](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-MC%20Series/RampUpDownToSelectedVelocity/RampUpDownToSelectedVelocity.ino) | A/B select one of four MSP-defined velocity selections. | ClearPath performs the ramp. The example may mirror external DI states to A/B, waits for input filtering, then waits for HLFB at target velocity or fault. |

## ClearPath-SD Series

| Example | Command pattern | Completion, homing, or notable dependency |
| --- | --- | --- |
| [MovePositionAbsolute](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-SD%20Series/MovePositionAbsolute/MovePositionAbsolute.ino) | Calls `Move(target, MOVE_TARGET_ABSOLUTE)` in step pulses, with configured velocity and acceleration limits. | The generator's reference starts at its program-start reference unless explicitly set. The helper waits for both `StepsComplete()` and asserted HLFB; homing is optional in this example. |
| [MovePositionRelative](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-SD%20Series/MovePositionRelative/MovePositionRelative.ino) | Calls `Move(distance)` in signed step pulses, relative to the end of the current move. | New commands can merge with a move already in progress. The helper waits for pulse completion and asserted HLFB; values are not automatically encoder counts unless MSP input resolution is configured 1:1. |
| [MoveVelocity](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-SD%20Series/MoveVelocity/MoveVelocity.ino) | Calls `MoveVelocity(signed_pulses_per_second)` with an acceleration limit. A zero target requests a ramp to zero. | The example waits for `AtTargetVelocity`, not positional move completion. HLFB must match its configured mode. |
| [DualAxisSynchronized](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-SD%20Series/DualAxisSynchronized/DualAxisSynchronized.ino) | Enables M-0/M-1, applies equal limits, then issues the same relative move to both connectors. | Waits for both generators and both HLFB signals. If either reports an alert, it commands abrupt stop on both. This coordinates calls and checks both axes; it does not implement a shared multi-axis trajectory planner. |
| [UserSeeksHome](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-SD%20Series/UserSeeksHome/UserSeeksHome.ino) | Seeks toward a hard stop at one velocity, slows to a second velocity, detects the configured HLFB transition, abruptly stops, backs off with a position move, then sets the reference to zero. | This is an application-managed homing sequence. It depends on a mechanically safe hard stop and configured homing torque; its polling loops have no timeout. |
| [MotorStatusRegister](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/ClearPath-SD%20Series/MotorStatusRegister/MotorStatusRegister.ino) | Periodically reads and prints the current status register and accumulated alert bits for each motor connector. | Diagnostic only: it does not enable a motor or command motion. Demonstrates how to inspect target, active, direction, enable, HLFB, limit, E-stop, and alert state. |

## High-Level Feedback (HLFB)

| Example | Feedback pattern | Notable dependency |
| --- | --- | --- |
| [AsgWithMeasuredTorque](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/High-Level%20Feedback%20(HLFB)/AsgWithMeasuredTorque/AsgWithMeasuredTorque.ino) | Reads HLFB state and, when a measurement is present, reports measured torque as a percent of peak torque. | Does not enable the motor or command motion. The ClearPath must be configured for the matching ASG-with-measured-torque mode and 482 Hz carrier. |
| [SpeedOutput](../.github/aiworkspace/clearpath/ClearPathModeExamples/examples/High-Level%20Feedback%20(HLFB)/SpeedOutput/SpeedOutput.ino) | Reads HLFB PWM and reports measured speed as a percentage of configured maximum speed. | Does not command motion. Requires the motor's Speed Output HLFB mode and matching carrier; the example notes mode availability restrictions. |

## Repeated Application Pattern

The examples that command motion generally follow this sequence:

1. Set the connector mode, HLFB interpretation, carrier, and applicable motion limits.
2. Configure external analog/digital inputs and initialize motor A/B states before enabling.
3. Request enable and wait for the expected ready feedback while also checking for motor faults/alerts.
4. Validate the requested value or selector, check fault/alert state, then write A/B levels, PWM, a trigger train, or a numeric move.
5. Honor MSP input-filter timing where A/B inputs are changed; wait for the feedback appropriate to that mode.
6. On fault, stop or reject subsequent motion. Optional automatic recovery is disabled by default in most samples because clearing a fault can re-enable motion.

Serial setup and status printing are for diagnostics and are not needed for the motion path. Several examples use unbounded polling loops; they demonstrate expected signals but do not provide a general timeout/recovery policy.

## Additional sketches

| Example | Inputs and units | Completion and faults | Requirements |
| --- | --- | --- | --- |
| [StepAndDirection](../.github/aiworkspace/clearpath/StepAndDirection/examples/StepAndDirection/StepAndDirection.ino) | Signed relative pulse distances and pulse/s, pulse/s² limits | Source waits for emitted steps, with optional HLFB; PM instead requires fresh measured settle completion | Explicit pulse-to-axis-count conversion; qualified motor feedback/output. No step pulse output is generated. |
| [FollowEncoder](../.github/aiworkspace/clearpath/EncoderInput/examples/FollowEncoder/FollowEncoder.ino) | Independent quadrature position counts or counts/s, direction inversion, index diagnostics | Continuous mode, not a finite move completion; source quadrature errors stop the example. PM additionally enforces source age and ownership | Separate command encoder and motor feedback, explicit transmission scaling, qualified resources and acquisition snapshots |

## PM capability outcomes

The table describes software support. All physical motion remains unsupported on
unqualified boards. Every row is checked by the PM host suite. Preset numbers
below are converted to zero-based software selectors; counts/scales are explicit
configuration, never copied implicitly from MSP. See [PM contracts and USB v1](pm_behavior_abstraction.md).

| Example | PM behavior/capability | Unsupported reason or semantic qualification |
| --- | --- | --- |
| Abs2PositionsHomeToSwitch | Position table + switch home | Requires qualified switch, motor feedback and output |
| Abs4PositionsHomeToHardStop | Four-entry position table available | Full example unsupported: hard-stop homing unqualified |
| Abs16PositionsHomeToHardStop | Sixteen-entry position table available | Full example unsupported: hard-stop homing unqualified |
| FollowDigitalPosition | Position-follow with command freeze | Requires qualified independent input; no PWM electrical compatibility |
| FollowDigitalTorque | Bounded current in mA | Independent speed/travel protection; no measured-torque claim |
| FollowDigitalVelocity | Continuous velocity ramp | Input scaling/polarity explicit; at-speed is nonterminal |
| FollowDigitalVelocityWithVariableTorque | Velocity with per-request current cap | Current cap can only lower commissioned limit |
| ManualVelocity | Checked delta to commanded velocity | No quadrature waveform generation |
| Move2IncrementsHomeToSwitch | Selected increment × checked repetition count + switch home | Trigger counts preserved; no enable-pin signaling |
| Move4IncrementsHomeToHardStop | Four-entry increment table available | Full example unsupported: hard-stop homing unqualified |
| PulseBurstPositioning | Relative position with primary/alternate speed | No pulse generation; measured settling replaces electrical completion |
| RampUpDownToSelectedVelocity | Four-entry velocity table and ramps | At-speed status keeps velocity mode active |
| MovePositionAbsolute | Absolute integer-count profile | Qualified reference and measured settling |
| MovePositionRelative | Relative to accepted commanded endpoint | Retry is idempotent; explicit replacement |
| MoveVelocity | Continuous velocity ramp | Fresh measured at-speed status; zero remains active velocity mode |
| DualAxisSynchronized | Unsupported | Independent dual feedback/output and common-start not qualified |
| UserSeeksHome | Unsupported | Example uses hard-stop homing; switch homing is a distinct supported capability |
| MotorStatusRegister | Lifecycle, target, feedback, limits, saturation and fault telemetry | No enable/motion needed |
| AsgWithMeasuredTorque | Unsupported torque estimate | Calibration unavailable; current telemetry remains available |
| SpeedOutput | Measured encoder velocity telemetry | No electrical HLFB output or implicit percent-of-MSP scaling |
| StepAndDirection | Relative integer-count profiles | Motion semantics only |
| FollowEncoder | Independent position/velocity follow | Qualified command acquisition and motor feedback; deterministic source-loss stop |
