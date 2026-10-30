# ODrive ros2_control Plugin

This package serves as a hardware interface to control ODrives from [ros2_control](https://control.ros.org/master/index.html).

It assumes that the ODrive is already configured and calibrated (see [docs](https://docs.odriverobotics.com/v/latest/guides/getting-started.html) for details).

**This is a work in progress** (see **Features**).

## Usage

For a high level usage example, see the [BotWheel Explorer ROS2 Package](../odrive_botwheel_explorer/README.md).

## Features

### Hardware timing diagnostics

`MARVIN_TRACING_MODE=off` disables hardware timing; `metrics` (default) and
`capture` enable the same fixed-size aggregates. A non-control reporter thread
logs `HARDWARE_TIMING` JSON (`hardware-timing.v1`) approximately once per second.
This first profiling slice uses the journal, not `/trace/metrics`, and creates
no ROS node or high-rate diagnostic messages.

`read`/`write` durations include mutex wait; `read_lock`/`write_lock` isolate that
wait. `sender` measures each actual setpoint send; the retained `pacing` field is
zero with the separate sender (there are no sleeps in `write()`). Frame counts
cover normal reads and setpoint sends; `tx_failures` counts rejected local sends,
not motor acknowledgements. Lifecycle requests are excluded. Export boundaries
are approximate because counters are exchanged separately. No history grows
with runtime duration. Off mode creates no reporter and takes no timing clocks.

### Separately paced commands

`write()` validates and encodes the latest targets into fixed per-axis slots. A
separate thread transmits one slot at a time, sleeping **outside** the CAN mutex.
At 12 axes and 50 Hz per axis, slots are nominally 1.67 ms apart. The sender uses
a nonblocking mutex attempt; busy/missed slots never generate catch-up bursts.
Rates are upper targets, not hard real-time guarantees. A newer hardware update
replaces unsent values; different axes can observe different update generations.
There is no FIFO of historical targets. Kernel-accepted frames cannot be recalled.

The latest revision uses steady-clock `sleep_until`: normal send work is
included in the slot interval rather than followed by a full slot sleep. Each
opportunity anchors a new deadline; this is not phase-locked catch-up scheduling.
An overrun waits another slot, and busy mutex attempts remain skipped. OS wakeup
lateness can still reduce throughput. This refinement passed local and Pi tests
and was installed with Marvin's runtime stopped on September 21. Its physical
validation is pending; it is not included in the measurements below.

`read()` still shares the transport mutex for brief serialization, but drains at
most 256 frames or until its 1 ms budget is observed after a frame. Processing one
frame and mutex wait can exceed that time; this is not a hard deadline. Watch
`rx_budget_hits`, `sender_busy_skips`, feedback freshness and real CAN counters.

Stop inhibits the sender and clears pending frames under the same mutex used
for transmission, before verified-IDLE requests. Mode switches discard old-mode
targets. Reactivation primes measured hold targets directly. A fresh `write()`
starts paced repetition; pending sender frames cannot replay across activation.
Stale hardware updates, non-finite targets or send failure latch a fault, inhibit
position/velocity/torque sends, request IDLE best-effort and return ERROR from
subsequent read/write callbacks. **This is not verified motor shutdown**: Rest must
still obtain fresh IDLE acknowledgements. No silent retry/resumption. ODrive
watchdogs remain necessary protection against process, transport or power faults.

Freshness measures hardware `write()` submissions, not new policy inference;
upstream policy watchdogs remain separate. Valid repeated targets are resent to
refresh the motor watchdog. Timing diagnostics being off does not disable these
safety clocks/checks. Validate watchdog settings, ramp fidelity, per-joint skew,
delivery latency and bus capacity before physical rollout. Deployed to Marvin on
September 21: supported testing measured 199–201 Hz controller updates and
423–472 total target frames/sec (roughly 35–39 Hz per motor, below the nominal
50 Hz). All 12 drives confirmed closed loop/error-free, with no CAN error-counter
increase in the 15-second window. This is not walking or hard real-time sign-off.

Build with `BUILD_TESTING=ON`; `ctest` exercises counters and the production
driver's fake-transport shutdown recovery with diagnostics enabled and disabled.

- Communicates over Linux SocketCAN
- Position Control (with optional velocity and torque feedforward)
- Velocity Control (with optional torque feedforward)
- Torque Control
- Automatic control mode selection (based on which Command Interfaces are claimed by the ros2_control Controller)
- Position, velocity and torque Feedback
- Multiple ODrives

**TODO:**

- Error feedback & error handling: If an ODrive disarms for some reason (e.g. undervoltage), the application that connects to ros2_control will currently not be notified.
- Other telemetry: Additional data like temperatures, DC voltage, etc. are currently not propagated through ros2_control up to the application.


## Parameters

For receive-overrun investigation, see the [offline evidence audit and matched
comparison protocol](docs/can-error-comparison.md). The accompanying
`scripts/analyze_can_echo.py` analyses saved JSON only; it cannot measure RX
errors or establish causality from position echoes.

See the [offline batch deadline review](docs/batch-deadline-review.md) for the
distinction between nominal cadence, snapshot age and submission spread. The
experimental batch cutoff is a pre-send check, not a hard delivery deadline.

Top level:

- `can`: Name of the CAN interface to run on
- `can_tx_rate_hz`: maximum nominal target rate per configured axis, default 50;
  finite range 1–200 Hz. Slots include axes with no claimed command interface.
- `can_command_stale_ms`: maximum age of the latest hardware submission before
  faulting at the next sender opportunity, default 250; finite range 20–1000 ms.

Per joint:

- `node_id`: `node_id` of the ODrive

## Command Interfaces

(from ros2_control Controller to ODrive)

- `position`
- `velocity`
- `effort` (aka Torque)

## State Interfaces

(from ODrive to ros2_control Controller)

- `position`
- `velocity`
- `effort` (aka Torque)

## Encoder freshness diagnostics

With `MARVIN_TRACING_MODE=metrics` (default) or `capture`, the existing background
reporter emits `ENCODER_TIMING` JSON once per second. No new ROS interfaces or
control rules are introduced. `off` skips kernel timestamp collection and sampling.

`encoder-timing.v1` has `window_s` and a `motors` array indexed by `node_id`:

- `queue_delay`: count/mean_ms/max_ms from Linux software kernel reception to
  decoding a valid finite encoder report.
- `age_at_read`: count/mean_ms/max_ms of the cached last valid report at the end
  of each hardware read, including its queue delay plus monotonic elapsed time.
- `frames`, `missing_stamp`, `invalid_clock`, `never_received_reads`, and
  `unavailable_age_reads`: windowed counters. Missing/invalid age is not zero.

Realtime-to-monotonic offset changes exceeding 5 ms and future timestamps
invalidate age; this is a diagnostic guard, not complete clock-step detection
(initial receipt, small jumps and clock changes hidden between samples remain
limitations). No motor-side acquisition timestamp exists in the encoder payload.
These metrics are not policy-consumed observation age. Shutdown/inactive samples
are included: use runtime phase/log timestamps when analyzing windows.

Storage is fixed per configured axis. Control-thread updates allocate nothing and
do not log; reporter reads/reset atomics without the CAN lock. Window boundaries
are approximate, like existing hardware timing counters. Freshness diagnostics
do not reject data, stop motors or alter CAN rates/watchdogs.

### Sender diagnostics

Hardware timing adds three count/mean_ms/max_ms aggregates in metrics/capture:

- `sender_wake_lateness`: actual wake minus requested sleep deadline, clamped to
  zero. Includes inactive wakes; select active windows when interpreting it.
- `target_age_at_send`: elapsed time from latest hardware mailbox commit to the
  send opportunity. Repeated writes refresh it even for the same policy action.
- `send_attempt_interval`: interval between consecutive frame attempts, across
  all axes (not per-motor frequency). Includes mutex-busy skips, resets on a
  no-frame opportunity or observed shutdown, and counts failed send attempts.

These are software timestamps, not CAN wire transmission or motor acceptance.
Existing `sender` duration and busy-skip/failure counters complement them.
No extra clock sampling for these diagnostics when tracing is off; no scheduling,
watchdog, target, rate or catch-up behavior changes. Export windows are approximate.
Full action-to-CAN correlation still needs explicit command identity across the
controller/hardware boundary; target value matching is not a valid substitute.

### Experimental phase-locked pacing (not enabled by default)

Hardware parameter `can_tx_pacing` accepts `relative` (default, unchanged) or
`phase_locked`. Invalid values reject initialization. The candidate advances an
absolute slot grid, skipping deadlines that would be closer than 90% of a slot
to the latest send opportunity. At twelve axes/50 Hz that floor is ~1.5 ms, versus
~1.667 ms in relative mode. This is an intentional relaxation of minimum spacing,
not proof that the MCP2515/ODrives tolerate it under all loads.

Long stalls never replay missed frames. Work overrunning a deadline waits at least
a full slot after completion; mutex-busy attempts still wait a full slot without
blocking or advancing motor order. Stale targets, failed sends, mode changes and
shutdown retain existing inhibition/IDLE verification. No spin-wait or priority
change. Timing off still controls diagnostics independently from pacing mode.

`test_pacing_experiment` exercises real mailbox code for both modes: constant and
periodic synthetic wake delays plus 100,000 opportunities per mode with variable
send work, long stalls and mutex-busy retries. Results are simulation, not hardware
performance predictions. Fake-driver tests exercise both modes with twelve axes
and tracing on/off. Live opt-in validation remains required before changing default.

`bounded_phase` is a second experimental candidate: instead of rounding the spacing
floor up to the next grid slot, it re-anchors at `max(next_slot, now + 0.9*slot)`.
The full-slot pause after overrunning send work remains. Default is still relative.
Mailbox/adversarial tests exercise all three modes; full fake-driver tests currently
cover relative/phase_locked only. Neither candidate is enabled on the robot.

### Coherent batch candidate (opt-in, physical validation not passed)

`can_tx_pacing=coherent_batch` snapshots one complete hardware-write generation
into preallocated storage at the start of each round. It attempts its frames
consecutively, without intentional inter-frame sleep, releasing/reacquiring the
transport mutex and rechecking inhibition/freshness between attempts. The next
round starts no sooner than one configured period after the previous round began
(20 ms at 50 Hz). A late round does not replay missed rounds. A mutex-busy attempt
retains the existing slot wait; lock reacquisition does not guarantee fairness.

New hardware writes update the next snapshot, not a partially sent round. A
partial round reaching one full period, or its snapshot reaching the stale limit,
faults/inhibits rather than continuing indefinitely. Send failure, Stop and mode
changes cancel pending frames; kernel-accepted frames cannot be recalled. A batch
is not an atomic transaction at the drives and software submission timestamps do
not prove wire time or simultaneous application. Hardware generations do not yet
carry policy-action IDs.

Batch mode uses no per-frame allocation, sleeps or logging in the transport lock.
It remains opt-in; default relative pacing and robot configuration are unchanged.
Unit tests cover mid-batch replacements, missing interfaces, expiry and stop;
fake-driver tests cover twelve axes, failure/IDLE recovery and tracing on/off.
Before live use, measure first/last submission and wire timing where possible,
feedback freshness, CPU, queue/send failures and CAN errors. Software batching
could increase short-term bus/SPI load even at the same average command rate.

Optional `can_tx_batch_gap_us` adds 0–1000 microseconds between batch slots,
measured from socket-send completion, with sleeping outside the transport lock.
Default 0 preserves compact batching; a nonzero value requires coherent_batch.
Configuration rejects non-finite/negative values and a gap-only round budget
`gap * (axes - 1)` reaching the round period. Actual send/lock/scheduling overhead
can still exhaust the period and trigger the existing fault. There is no catch-up
burst after a late wake. Tracing off retains identical gap/deadline behavior.
For twelve axes, 500 us adds at least 5.5 ms between first/last slots, plus send
work and scheduling. This is an experimental starting point, not a proven fix.

Two compact-batch physical tests aborted on RX-error increments; relative remains
the default. Paired software echo spans averaged 20.635 ms relative / 3.241 ms
compact batch, but are not motor-application times. The spaced option has not
been physically validated. See Marvin wiki services/ros2-runtime for evidence.

In metrics/capture, HARDWARE_TIMING includes a `batch` object:

- `started`, `completed`, `aborted`: rounds started, fully submitted, or cancelled
  by fault/stop/mode switch. A last-frame send failure aborts, never completes.
- `submitted_frames`: frames successfully accepted by the socket, including
  partial rounds. Missing command interfaces are skipped, not counted as frames.
- `submission_span`: count/mean_ms/max_ms from snapshot creation before the first
  frame to completion of the last slot, for completed rounds only. This includes
  send work and lock/scheduling delays, not downstream wire/actuator latency.

An empty-interface round can complete with zero submitted frames; it is not proof
of twelve motor commands. Counts reset on export and may cross window boundaries;
started need not equal completed plus aborted within one report. Off skips batch
counter/timing updates; cancellation and freshness checks remain enabled.

Sender inhibition always logs `reason=...`, independently of tracing:

- `producer_stale`: no fresh committed hardware targets within the stale limit.
- `batch_deadline`: an unfinished batch reached its round period.
- `snapshot_stale`: frozen batch targets reached the stale limit, even if newer
  hardware writes are available.
- `non_finite_target`: enabled command input contains NaN/infinity.
- `send_failure`: the setpoint socket-send call failed.

The first detected reason is latched through inhibition and later faults, cleared
on activation or explicit fault clearing after confirmed IDLE. Existing check
order is retained: producer staleness, then batch deadline, then snapshot age.
This identifies the guard that fired, not the underlying scheduler/transport
cause. Logging does not prove IDLE; Rest must still verify motor acknowledgements.

`HARDWARE_TIMING.send_execution` pairs steady-clock elapsed time with
`CLOCK_THREAD_CPUTIME_ID` around each setpoint send. It exports elapsed, CPU and
`off_cpu_estimate` durations; a separate `slow_*` subset includes valid paired
samples with elapsed time >=2 ms. The threshold is diagnostic, not a safety limit.
CPU time includes user/kernel execution charged to the sender thread. The
elapsed-minus-CPU estimate includes time not charged to that thread and boundary
overhead; it cannot distinguish scheduler preemption from kernel waiting, nor
identify the competing process. The SocketCAN socket already uses SOCK_NONBLOCK;
elapsed send time does not measure on-wire transmission or motor application.

Failed/inconsistent CPU-clock samples increment `invalid_cpu_samples` and retain
only elapsed time, never fabricate zero CPU time. Counters are preallocated;
there is no per-frame logging/allocation. Metrics mode adds two thread-CPU clock
reads and paired counters per send; tracing off skips the diagnostic clocks and
counters. Measure observer overhead before comparing runs. Export boundaries are
approximate as for other timing counters; maxima may belong to different calls,
so never subtract independently exported maxima to infer off-CPU time. Existing
wake lateness and transport-lock busy-skip counters complement these samples.

For an approved live comparison, `python3 scripts/capture_can_echo.py --seconds 20`
is a receive-only observer of locally echoed Set_Input_Pos frames. It never sends,
caps records at 40000, and reports software timestamps and capture loss indicators.
Zero echoes while stopped is expected. Driver echo semantics must be established
before interpreting timestamps: they are not hardware wire timestamps or motor
acknowledgements. Node/order/payload capture does not provide policy-action IDs.

For a synchronized diagnostic, add `--include-encoder-feedback`. This opt-in mode
also receives remote standard, non-RTR Get_Encoder_Estimates frames (0x09). It
retains local Set_Input_Pos echoes (0x0c), labels each frame type, and emits
`can-timing-observer.v1`. The echo-only analyzer intentionally rejects that schema.
The default mode retains its original frame selection and v1 echo schema.

Combined mode stores bracketed monotonic/wall-clock samples approximately once
per second and at start/end. These support clock alignment and clock-jump checks,
not physical encoder sample timestamps. Kernel receive times are software receipt
times. Local echoes are not motor acknowledgements or verified wire completion.
Repeated identical payloads cannot identify a unique policy action.

Use a short bounded interval, initially 20 seconds, alongside the reference bag
and optional native diagnostics on the robot host. At 12 controllers and 50 Hz in
each direction, approximately 24,000 frames fit under the unchanged 40,000-record
cap. Longer captures can drop later records. Inspect missing timestamps, truncated
ancillary data, socket drops, and capture-limit drops before analysis. Buffered
records export at normal completion; forced termination or power loss can lose them.
No mode transmits frames, changes the CAN interface, or activates motion.
Validate combined recording overhead with motors inactive before an operator-approved
physical test. This extension has offline parser tests, not deployed Pi validation.

Analyze a completed combined capture without CAN or ROS access:

```bash
python3 scripts/analyze_can_timing.py /path/to/can-timing.json
```

The report separates each node's position echoes and encoder feedback. It checks
payloads, finite values, timestamp ordering, clock brackets, capture losses, and
paired node streams. Positions remain motor revolutions and feedback velocities
remain revolutions/second. It does not guess gearing, direction, or joint offsets.
The default clock-offset tolerance is 1 ms. Clock jumps or wide clock brackets
prevent the cross-stream readiness flag, as do missing timestamps or reported drops.
An empty capture or unpaired streams are not ready for response analysis.
Readiness is not a motion approval, hardware health check, or proof of zero loss
before the observer socket. The report does not identify filter coefficients,
assign policy action IDs, or calculate motor latency.

Captures also include host name, Linux boot ID (null if unavailable), and the
recorder source SHA-256, calculated before measurement. Check these when joining
other recordings. Monotonic timestamps from different hosts or boots are not
directly comparable. The analyzer preserves this identity but does not authenticate
it or validate identity against another file.

Offline pipeline tests run the full recorder CLI against a receive-only fake
socket, then analyze its JSON. They check default echo compatibility, combined
filters, timestamp ancillary data, truncation, and socket-drop propagation.
They do not validate kernel/driver echo semantics or Pi performance.


## Measured hold before activation

Activation requires finite encoder positions received within 100 ms for every
axis. Freshness uses the kernel CAN receive timestamp, including time spent in
the receive queue. Timestamp collection remains enabled when tracing is off.
Missing, stale or nonfinite feedback rejects activation before any motor request.

The driver sends all measured position targets, with zero feedforward, before
it sends any CLOSED_LOOP_CONTROL request. A failed prime send rejects activation.
The position conversion is the same as normal command writes. Reactivation takes
a new measured hold, so a joint moved while inactive does not return to an old target.
These sends prove transport submission, not motor acknowledgement or physical tracking.
A failed closed-loop send reports activation failure. Some drives may already be
holding their primed targets; the driver does not issue automatic IDLE on that path.

Marvin enables `require_command_receipt=true` in the physical hardware description.
This exports each joint's `command_receipt_monotonic_s` command interface and requires
an active position command interface. The paired traced forward controller enables
`export_command_receipt=true`. It records host steady time when a command callback
receives a message and transfers that time with its position vector. Repeated update
cycles do not renew receipt time. Until receipt is later than activation priming,
the driver repeats the measured hold with zero feedforward. Missing receipt retains hold.
Both components use the same host clock. This blocks retained controller commands;
it cannot distinguish an old DDS message first delivered after activation.
Isaac and Gazebo descriptions do not claim this additional interface.

`test_activation_driver` uses the production driver with a fake CAN transport.
It checks prime-before-enable ordering, nonzero poses, failed priming, stale queued
feedback, missing/nonfinite feedback, retained commands and moved-joint reactivation.
The same checks run with tracing disabled. No physical motion is established by them.
