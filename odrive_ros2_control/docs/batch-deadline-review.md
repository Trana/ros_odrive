# Batch deadline review — 2026-09-24

Status: offline design review; **no production behavior or robot settings changed**.
The experimental coherent/spaced sender remains opt-in. Relative pacing remains
Marvin's restored configuration. The tests below are not hardware validation.

## Issue #7 implementation update — 2026-11-01

The `tickets/7-prepared-recorder-hold` branch implements the preferred separation
as an opt-in native fallback. It does not enable the fallback or add production
limits. Missing configuration preserves the existing IDLE response.

`PacedCommands` now retains the frozen command generation, sanitized position-only
frames, target positions, source time, batch start and next-axis cursor when the
first fault is `batch_deadline`. The ODrive hardware owner can admit that batch to
`SupportFallback` only when all reviewed limits are present, all axes use position
mode, the active hardware epoch and generation authorize the batch, and every axis
has current closed-loop, heartbeat, error and encoder evidence.

The fallback finishes only the same frozen generation within the configured
completion span. It then repeats the position-only frames. It fences all newer
hardware writes and never resumes motion automatically. An atomic Stop latch is
set before lifecycle shutdown waits for the CAN mutex and is checked before each
fallback frame. Stop, send failure, stale or invalid data, motor error/disarm,
evidence expiry and hold expiry retain the verified-IDLE path.

### Continuous healthy hold — 2026-11-02

The user approved removing automatic IDLE solely because a hold-duration timer
expired. The fallback now repeats the same frozen position-only targets while
motor observations, tracking and per-axis submissions remain within the reviewed
bounds. Explicit Stop, transport failure, motor error/disarm, stale evidence and
tracking failure still terminate fallback. It never resumes locomotion automatically.

`can_hold_max_duration_ms` is obsolete and rejected when fallback is enabled, so
an older configuration cannot silently acquire different expiry behavior. The
remaining five required limits are unchanged and still need production review.
No robot configuration is enabled by this source change. The previous references
to hold-duration expiry describe the earlier implementation.

The full Jazzy native suite passed all 21 tests, including a fake-CAN driver hold
lasting beyond the old one-second test timeout. A virtual-time test holds for one
minute, then checks Stop. Post-hold feedback, tracking, motor and send failures
still select IDLE. These tests do not establish physical readiness.

Each joint exports support state, evidence level, generation, last successful
submission, tracking error and the aggregate confirmation result. Socket acceptance
is `submitted`, not `observed`. Confirmation requires current motor and tracking
evidence for every axis and never claims balance.

The follow-up correction binds position-support permission to the batch when that
batch starts. A 200 Hz producer write can no longer revoke the in-flight 50 Hz
batch by replacing a mutable latest-generation value. `due()` also identifies the
axis and batch. The sender records each accepted frame in a pending ledger and
publishes the ledger only after every axis in one generation is accepted. A partial
replacement does not make the last complete evidence disappear.

Normal streaming now exports last-complete per-axis submissions even when fallback
is disabled. Without a reviewed policy, these facts stay `submitted`; they cannot
become `observed` or set `hold_confirmed`. This corrects the evidence gap exposed by
the operator's observation without turning test values into production limits.

Offline fake-CAN coverage includes the 25 ms sender stall that caused a partial
batch deadline. The opt-in test confirms continued position-only sends and no IDLE
request, then confirms explicit deactivation still uses verified IDLE. All fatal
paths remain fail closed. Pi timing measurements, reviewed configuration, simulation
and secured-stand tests remain required before enablement.

## What the current guard actually guarantees

Source: `include/paced_commands.hpp` (`due`, `finish_submission`, `wake_after`)
and `src/odrive_hardware_interface.cpp` (`sender_step`).

- At 50 Hz, the nominal round interval is 20 ms. The same interval is also the
  partial-batch cutoff, measured from the actual first send opportunity.
- A late first wake can start normally if the hardware producer is fresh.
  A late partial round faults even if newer writes are waiting.
- The cutoff is checked **before** each send. A last send starting just before
  20 ms can return after 20 ms and still increment `completed` today.
- Completion means local socket acceptance, not delivery, motor execution or
  simultaneous actuation. Previously accepted frames cannot be recalled.
- A busy transport mutex skips the mailbox check; an unscheduled sender cannot
  enforce a wall-clock deadline. Fault detection therefore occurs when execution
  and lock access resume, not necessarily at the limit itself.
- A hardware `write()` refreshes producer age even when the policy action is
  unchanged. Frozen snapshot age and upstream policy freshness are separate.
- Fault inhibits commands and requests IDLE best-effort. Verified shutdown still
  requires fresh IDLE responses; this is not an independent safety controller.

## Separate these quantities

| Quantity | Meaning | Intended treatment |
| --- | --- | --- |
| Cadence | Desired round start interval (20 ms at 50 Hz) | Scheduling goal; count misses, never replay a backlog |
| Producer age | Time since latest hardware write | Existing freshness fault, not policy-action age |
| Snapshot age | Time since the frozen round's hardware write | Independent maximum; newer writes cannot reset it |
| Submission span | First opportunity to last send return | Independent bound, checked before and after sends |
| Per-axis update age | Time since that axis's last successful submission | Detect starvation even across replaced rounds |
| On-wire spread | First-to-last observed CAN frame of a round | Measure separately; socket timing cannot guarantee it |

## Options and recommendation

1. **Keep the one-period fault:** conservative and simple, but conflates a missed
   performance target with an unsafe age/spread. Keep this implementation until a
   replacement's limits and failure behavior have been reviewed and tested.
2. **Separate cadence from age/spread:** preferred next experimental design.
   Preserve frozen rounds; allow a missed cadence only while independent bounds
   remain valid. Check before every send and after each return, including the
   last. A post-send violation must latch a fault and be reported separately from
   a successful batch, although it cannot undo that send. Retain stale-producer,
   stop/mode invalidation and send-failure handling. Do not compress gaps or send
   missed rounds to recover rate. Track missed cadence, late completion and
   per-axis update age separately from delivery acknowledgements.
3. **Drop a partial round and restart the latest:** not recommended as a blind
   recovery. Earlier motors already received old targets; repeated restarts at
   axis zero can starve later motors. Rotation alone does not establish acceptable
   generation mismatch or joint timing. Any replacement strategy needs its own
   per-axis fairness and freshness design.

Do **not** select a new bound merely because training used a 30 ms actuator
delay. A fixed trained delay is not equivalent to variable inter-joint skew,
stale observations or missing commands. The last documented read-only check on
September 21 found all twelve drive watchdogs enabled with a **15 s** timeout;
that is not a substitute for a reviewed short command-age guard. No watchdog
configuration was changed or re-read from the offline robot for this review.

## Offline evidence and remaining work

`test/test_batch_deadline_review.cpp` characterizes the actual mailbox's late
start, frozen snapshot, exact cutoff, late final completion, no immediate
catch-up and stop behavior. Its separate decision table uses **synthetic**
30 ms span / 60 ms age values only to demonstrate independent boundaries; they
are not proposed robot settings. A constructed drop/restart schedule demonstrates
starvation. It is not a new sender or an OS/transport stress test.

Before replacing the guard: agree acceptable per-axis age and joint skew,
including upstream policy freshness and watchdog relationships; implement an
opt-in state machine with completion checks and per-axis metrics; test sustained
and repeated delays, lock contention, transport failures and stop races using the
fake driver with tracing on/off. Then compare CAN echo timing and feedback under
operator-approved supported tests. No threshold has been approved by this review.
