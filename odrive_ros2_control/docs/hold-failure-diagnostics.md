# Hold failure diagnostics — issue #7, 2026-11-06

The driver emits one `CAN_HOLD_FAILURE` record when a sender fault reaches the
IDLE path. It preserves the original sender fault and the first recorded failed
hold check. This is diagnostic only: eligibility, limits, Stop priority and IDLE
behavior are unchanged. It does not establish moving-policy recovery safety.

Fields:

- `phase`: admission, finishing_batch, holding, or streaming.
- `check`: failed age, completion, tracking, motor state/error, position mode,
  target validity, batch identity/shape, missing frame, transport send, or explicit
  Stop check. Disabled hold and unavailable recovery batches are also identified.
- `node`, `joint`, `axis`: affected CAN node, joint name and zero-based axis index.
  Batch-wide failures use -1 / none.
- `actual`, `limit`: measured value and bound. The check name identifies ms or rad.
  Boolean checks use 0/1. Missing or inapplicable values are `nan`.
- `target_rad`, `position_rad`: frozen target and measured joint position when
  the failed observation check supplies them.
- `snapshot_age_ms`, `batch_age_ms`, `activation_epoch`, `generation`,
  `snapshot_id`: frozen command identity and ages at the failed check.
- `sender_reason`: original sender fault, retained if hold subsequently fails.
- `hold_enabled`: configured opt-in flag.
- `captured_monotonic_s`: driver shutdown-evidence capture time, independent of
  the later journal emission timestamp.
- `dropped_events`: additional events omitted while the single pending slot was
  occupied. The oldest pending event is retained.

The sender copies a fixed-size snapshot into a single-producer/single-consumer
atomic slot. A reporter polls at 100 ms and formats the message without holding
the transport mutex. The reporter runs even with `MARVIN_TRACING_MODE=off`.
Ordinary explicit Stop without a sender fault produces no extra fault record.
Cleanup drains pending evidence; abrupt process termination can still lose it.
Existing sender error messages remain unchanged. No per-frame logging was added.

When several gates fail together, this record identifies the first recorded
failure, not every failed condition or the electrical/scheduling root cause.
Later shutdown cannot overwrite that first check. Reset starts a new diagnostic
lifecycle. A `batch_deadline` message alone still does not establish its cause.

## Verification

Build `odrive_ros2_control` with `BUILD_TESTING=ON`, then run CTest. The
`support_fallback` test checks admission tracking, heartbeat/error/encoder ages,
motor state/error, snapshot/completion bounds, ongoing tracking loss, submission
expiry and first-cause preservation. `hold_failure_diagnostic` and
`hold_failure_diagnostic_off` inject a 25 ms fake transport stall against a 10 ms
hold completion bound. They require exactly one record with the admission check,
configured limit and original batch fault, emitted outside the sender thread.
These tests use fake CAN only. They do not activate physical motors.
