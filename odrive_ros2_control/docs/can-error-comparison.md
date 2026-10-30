# CAN receive errors: offline evidence and comparison protocol

Reviewed 2026-09-27. No SSH, CAN socket, service or robot configuration changes.
This is preparation, not permission to activate motors or relax guards.

## Evidence audit

| Evidence | Availability in this review | Finding | Limitation |
| --- | --- | --- | --- |
| September 22 passive baseline | Local `/tmp/marvin-can-baseline-2026-09-22.json` | 40.00018 s, 48,515 received frames; RX errors and RX overruns both +2; TX frames remain zero | Different day/load/state from active trials |
| First compact test | Wiki summary; raw files on robot | RX errors 36→37, stopped around 11 s | No exact counter event time in selected hardware windows |
| Paired relative/compact test | Wiki summary; raw files on robot | Relative: 15 s, RX errors 55→55. Compact: ~8.6 s, 56→57 | Background 55→56 between runs; unequal exposure; one pair only |
| 500 us spaced batch | Local `/tmp/spaced-batch.ObdluA/echo.json`; counter/fault summary in wiki | 609 echoes: 50 complete rounds and nine partial frames; recorded RX errors unchanged at 82 | Only ~1.05 s before sender inhibition; generic fault reason at that revision |
| Later explicit-fault spaced tests | Wiki summary; raw files on robot | `batch_deadline` faults with RX errors unchanged at 105 and 128 respectively | Not compact (zero-gap) runs and not reliability passes |

The passive capture has twelve IDLE heartbeats with error value 16777216; it is
not an error-free motor-state baseline. Its recorded RX-overrun deltas nonetheless
occur with TX counters zero before and after. Node 22 has two observed encoder
gaps above 15 ms (maximum 19.991 ms); other nodes have none. This is consistent
with lost feedback, but the summary lacks per-event correlation to prove which
overrun lost which frame. Counters are events, not an exact count of lost frames.

Conclusion: compact sending is **not necessary** for receive overruns to occur.
Whether it increases their incidence remains unresolved. Historical active trials
report aggregate RX errors; do not automatically relabel every increment as an
overrun without the corresponding detailed counters.

Raw paired captures to retrieve later, without rerunning motors first:
`/home/mikael/Development/backups/coherent-batch.eLLgsP/`:
`comparison-relative/` and `comparison-batch/`, each containing `echo.json`,
`supported-metrics.jsonl`, `hardware.log`. The same root holds the first trial.
Other roots: `spaced-batch.B9RI9r`, `spaced-diagnostic.2sCIPb`,
`send-execution-live.dFJROG` under the same robot backups directory.
Do not imply those raw files were re-read during this offline review.

## Reproducible local echo analysis

From `ros2/ros_odrive`:

```sh
python3 odrive_ros2_control/scripts/analyze_can_echo.py /tmp/spaced-batch.ObdluA/echo.json
python3 -m unittest discover -s odrive_ros2_control/test -p test_analyze_can_echo.py -v
```

Only Python standard library is needed. The analyser opens local JSON files;
there are no network, CAN or subprocess calls. Optional `--start-ns` / `--end-ns`
select a half-open Unix-nanosecond interval; `--nodes` sets the explicit motor
order. Keep boundaries tied to phase timestamps, not selected for favourable
results. Missing timestamp records break grouping; timestamp regression is an
error. It reports capture-level quality separately from the selected window.

Local spaced capture SHA256:
`d273df936799b17f20bfcd55aadd06a6ca3471517aa07a78816288c586f12bfe`.
Recomputed complete-round software echo span: mean 9.782959 ms, maximum
18.919025 ms, p95 **13.969219 ms using nearest-rank**. The historical wiki value
13.641 ms uses an unspecified percentile convention; do not silently overwrite
or compare percentiles across methods. Complete/partial counts, mean and maximum
match the historical summary. The capture contains no receive-error counters or
encoder messages. Echo interarrival gaps are NOT sensor age or encoder gaps.
Temporary paths may disappear; preserve/retrieve original evidence before future
comparisons rather than depending on `/tmp` as durable storage.

## Future hardware comparison protocol

### Prerequisites

- Fresh operator confirmation of power, stand support and proximity before active
  trials. Preserve existing shutdown/error guards, verify twelve IDLE afterwards.
- Retrieve saved evidence first. Record exact source/library/config identities,
  policy, node order, feedback intervals, heartbeat rate, tracing and observer
  settings, voltage, temperature/throttling and workload for each trial.
- Confirm counter collector exposes RX errors **and** RX overruns, RX drops,
  TX errors/drops, packets and available CAN controller error/state counters.
  Missing counters are unavailable, not zero. Keep protocol errors, SocketCAN
  observer drops, hardware overruns and sender faults separate.
- Capture boot ID, interface index and monotonic sample timestamps. Reject deltas
  across reboot/interface restart/counter decrease. No resets within trials.

### Stage 1: operational comparison at 100 Hz feedback

Baseline: motors IDLE, runtime stopped, same observers and feedback configuration.
Then A = restored relative pacing, B = coherent batch with zero gap, both nominal
50 Hz commands and unchanged 100 Hz feedback. Do not use 500 us spaced mode as B.
Start with existing bounded 15 s active windows. Use a predeclared balanced order,
for example AB then BA, with inactive baseline intervals and verified cleanup
between trials. A guard abort ends the trial and needs review; never automatically
retry or extend it to obtain equal time. Repetitions require operator approval.

Record startup/readiness, activation request, verified active, first target send,
fault/abort, Rest and verified IDLE as separate events. Sample counters periodically
with identical overhead and at phase boundaries. A counter jump is bracketed by
sample times, not an exact error timestamp. Exclude cleanup/post-fault traffic from
active exposure only when boundaries support it; otherwise mark it mixed.

Report every run, not just complete runs: actual active exposure, received frames,
counter deltas, abort reason/time, per-axis feedback gaps/age, command rate and
echo spans. Calculate events per second and per million received frames only from
aligned counter/time windows. These are descriptive event rates, NOT a probability
of losing a frame. A zero-event 15 s sample does not establish reliability; the
historical passive sample already shows how sparse events can be. Do not combine
deadline-aborted and completed runs into a simple pass percentage.

### Important confound: actual sending rate

Existing relative sending achieved ~44.5 Hz/motor versus ~49.4 Hz for compact
batches. Equal configured rate is not equal offered load. Stage 1 therefore tests
the two complete operating modes, not burst shape alone. If a repeatable difference
appears, separately design an actual-throughput-matched comparison (fresh review,
same feedback and workload), documenting changed rate settings. Do not claim
burst-shape causality from stage 1 or quietly tune one mode mid-comparison.

### Stage 2: feedback-rate hypothesis

After stage 1 review, compare 100 versus 50 Hz encoder feedback first while IDLE,
then under approved supported operation if appropriate. Preserve heartbeat,
command scheduling and guards. Confirm effective per-node feedback intervals.
Reducing traffic may help, but assess observation age and policy behaviour before
adopting it. Changing feedback and pacing together is not an isolating test.

## Decision gate

### September 28: explicitly bounded diagnostic allowance

After the strict relative baseline aborted on a single overrun, the operator
requested a bounded allowance instead of first-error abort. Test-only helpers
`scripts/can_diagnostic_guard.py` and `scripts/can_feedback_observer.py` implement
strict-by-default monitoring. Explicit bounded mode allows at most two isolated
overruns in the 15-second trial. Two in one rolling second abort (retention adds
50 ms conservatively for sample-time uncertainty); the third total also aborts.
This supersedes first-overrun abort only for explicitly selected diagnostics,
not production or prior test results.

Every monitored motor must have kernel-timestamped encoder feedback younger than
50 ms. Missing metadata, observer socket loss, observer scheduling gap >=50 ms,
clock mapping drift >5 ms, counter resets/identity change, unexplained RX errors,
TX errors and interface drops abort. Pure RX-overflow CAN error notifications
are delegated to the detailed counter budget; other CAN error frames, including
bus-off, abort. Heartbeat/motor-state and Rest cleanup guards remain in the
supported harness. These are conservative supported-test criteria, not validated
operating limits or a hard-real-time safety mechanism. An observer sees reception,
not proof that the policy has consumed the latest frame. Runtime's production
guards and hardware watchdogs remain separate and unchanged.

Unit tests (no hardware/socket opened):

```sh
python3 -m unittest discover -s odrive_ros2_control/test -p 'test_can*.py' -v
```

Twenty tests cover budget/burst boundaries, stale/missing feedback, scheduling
gaps, counter resets, non-overrun faults, kernel age vs dequeue age, bus-off and
observer drops and the normal-frame filter mask. The real motion harness is retained with trial evidence rather
than made an automatically executed test.

Idle-watchdog preflight clarification: explicit `--allow-idle-watchdog` in the
supported harness permits only fresh IDLE heartbeats with error 0 or exactly
0x01000000. This does not clear faults or disable/feed a watchdog. Normal plugin
activation requests CLOSED_LOOP_CONTROL; ODrive documents that this feeds the
watchdog and that active error flags clear when their condition disappears.
The option does not permit watchdog flags during active operation, nor any other
idle fault or mixed bitmask. Fresh post-activation heartbeats still require state
8/error zero. Historical queued pre-activation heartbeats are not active evidence.
Two added unit tests cover the narrow idle allowance and stale/missing states.

September 28 harness corrections and baseline: open the observer after the
blocking initial ROS status query, not before (queued preflight frames overflowed
the diagnostic socket). Remove inline whole-process CPU scans from the guarded
loop: one scan coincided with a 101.8 ms monitor gap and correctly aborted the
first activation after 1.10 s, with no new CAN errors. Neither case establishes
a robot feedback failure. Keep the 50 ms guard unchanged; collect CPU separately
in future, if needed. The corrected `relative-no-cpu` trial completed 15 s with
RX errors/overruns unchanged at 79, no TX errors/drops, verified twelve IDLE on
Rest, and stopped runtime. All 28 Python diagnostic tests pass.

Sources:
- https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#watchdog
- https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.active_errors
- `src/odrive_hardware_interface.cpp`, `on_activate()`.

No new physical trial is needed merely to establish that passive overruns exist;
the local baseline already establishes that. Retrieve the paired raw captures and
check counter/time alignment next. If still inconclusive, execute the predeclared
comparison with unchanged guards. Neither compact default promotion nor deadline
relaxation follows from this review.


### September 28 bounded active comparison result

After the harness corrections, relative-no-cpu completed 15 s with RX
errors/overruns 79→79. Compact-no-cpu activated successfully, then aborted after
5.804 s with errors/overruns 85→88 (third overrun exceeded the two-event budget).
No TX errors or interface drops increased. Both returned to Rest with all twelve
ODrives verified IDLE. Runtime is stopped; original physical xacro restored and
byte-compared. No feedback/watchdog settings or production guards changed.

Aligned local TX echoes: relative mean twelve-frame span 22.251 ms (p95 30.943),
compact 3.512 ms (p95 4.921). Actual per-motor send rates differ: ~40.9 vs ~48.9 Hz.
These are software echoes, not motor application timestamps. Captures report no
socket drops/missing timestamps. Compact's aborted exposure is not a successful
reliability trial. A single ordered pair, differing actual TX rates, and passive
counter increases prevent a causal conclusion; keep relative as default. Next
experiment should separate actual send rate from batching, not increase the
error budget. No automatic further motor trials.

Evidence: local marvin-can-bounded.PbR3LR/{relative-no-cpu,compact-no-cpu}; robot
/home/mikael/Development/backups/can-bounded.BLOixM with identical directory names.

### Rate-control follow-up protocol

Use existing `can_tx_rate_hz=41` with `coherent_batch`, zero batch gap, for one
15 s bounded supported trial. This targets the previous relative run's measured
40.9 Hz, not its configured 50 Hz. Accept approximate rate matching only if the
measured per-node echo rate is within 5% of that reference; report actual rates.
Do not increase the configured relative rate to force a match. Keep encoder
feedback, policy, tracing, observer, watchdog and stale-command limits unchanged.
The existing batch deadline equals its period, hence changes from 20 to 24.39 ms
at 41 Hz; this is a documented secondary difference, not a new independent safety
setting. Monitor freshness/scheduling bounds and overrun budget remain unchanged.

Stop on any guard and restore the original xacro after stopping runtime. A single
trial cannot establish causality or justify a default change. Passive counter
growth 88→94 between trials occurred with motion runtime stopped and TX unchanged
at 14,254; record this background activity rather than attributing it to batching.

### September 28 approximately rate-matched compact follow-up

One supported compact trial with `can_tx_rate_hz=41` measured 39.872 Hz/node
against the earlier relative baseline's 40.948 Hz (2.6% lower, within the
predeclared 5% tolerance). It aborted at 7.568 s on the third isolated receive
overrun: counters 95→96 at +1.709 s, →97 at +4.981 s, →98 at +7.568 s.
Counter-read event brackets were 18.38, 19.30 and 18.04 ms; these are not exact
frame-loss timestamps. No TX errors/interface drops increased. Mean ordered
twelve-frame software-echo span was 3.454 ms, p95 5.056 ms, maximum 10.241 ms
(302 complete groups). Capture reported no timestamp loss or socket drops.

The higher compact send rate is therefore not necessary for reproducing the
overruns. This strengthens concern about compact sending but still does not
prove batching caused them: passive overruns exist, trials are short/ordered,
the baseline was earlier rather than interleaved, and the native batch deadline
changed from 20 to 24.39 ms with the configured period. No reliability-rate
estimate or default promotion follows from these censored samples.

Rest verified all twelve IDLE at Unix 1790604404.724092396. Runtime inactive,
MainPID=0; original physical xacro restored and byte-compared. No further motor
trial, feedback adjustment, watchdog change or error-budget increase. Keep
relative default. Next useful isolation is a separately reviewed passive
feedback-load comparison, with original rates restored afterward; lower feedback
would trade reception load for older policy observations and is not yet selected.

Raw evidence and configuration: local marvin-can-bounded.PbR3LR/compact-41
and compact-41.xacro/run-compact-41.sh; corresponding robot files under
/home/mikael/Development/backups/can-bounded.BLOixM. The same 28 diagnostic
Python tests pass; shell syntax checked. No production C++ code changed.

### September 28 temporary 50 Hz feedback test

Operator authorized supported testing. Existing motor-controller manager services
read all twelve controllers IDLE with encoder period 10 ms and saved originals.
The diagnostic client needed ROS_LOCALHOST_ONLY=1 to discover that manager;
the initial failed discovery made no writes. Changed only
axis0.config.can.encoder_msg_rate_ms to 20, verified all readbacks, and measured
50.029–50.040 Hz per node over a five-second passive capture. No configuration
save, heartbeat/watchdog/policy change, or direct competing CAN writer.

Normal relative sender trial aborted after 4.681 s on the unchanged 50 ms monitor
scheduling guard, NOT a CAN-error budget. RX errors/overruns stayed 111→111;
TX errors and interface drops remained zero. Counter sampling intervals were
at most 21.965 ms, but there was a 64.813 ms interval between the final active
counter event timestamp and the abort event. This brackets a stall after counter
timestamp creation; it does not distinguish synchronous JSON/log output,
descheduling or another intervening delay. Do not attribute it to stale encoder
data or claim the 50 Hz comparison passed. Compact test was not run.

Rest verified all twelve IDLE at Unix 1790605109.471469559; service inactive,
MainPID=0. Original xacro restored/compared and all twelve encoder periods restored
to 10 ms with verified readbacks. All changes were temporary. Evidence:
marvin-can-bounded.PbR3LR/feedback50-relative, feedback-original.json,
feedback-rate.py, verify-feedback50.py and run-feedback50.sh; matching robot files
under /home/mikael/Development/backups/can-bounded.BLOixM.

Next: instrument monitor execution boundaries or buffer diagnostic event output
outside the guarded loop, without relaxing the 50 ms guard; validate idle first
before another supported relative/compact pair. No conclusion yet about whether
50 Hz feedback resolves the receive overruns.

### September 28 buffered monitor and completed 50 Hz pair

Diagnostic-only change: buffer timestamped events (10,000 cap), raw metric
strings (2,000 cap), and loop-stage timestamps (5,000 cap), flushing after Rest.
Any buffer exhaustion aborts. This removes synchronous per-counter JSON/output
and metric parsing from the monitored loop without changing the 50 ms guard.
Stage timestamps cover ROS spin, CAN drain, counter reads and guard check.
Three new extracted-function tests plus 28 existing diagnostic tests pass.
The earlier stall's precise cause remains unproven; buffering loses unflushed
evidence if the process is killed outright and is not a hard-real-time solution.

Idle-only 50 Hz check completed 15 s (1,053 steps); maximum check interval
19.153 ms. Then both supported active trials completed 15 s:

| Sender (configured 50 Hz) | Actual TX Hz/node | RX overrun delta | Mean / p95 twelve-frame echo span | Max monitor interval |
| --- | --- | --- | --- | --- |
| Relative | 42.584 | 1 (113→114), at +13.123 s | 21.545 / 28.017 ms | 20.508 ms |
| Compact | 48.969 | 1 (114→115), at +10.108 s | 3.222 / 4.382 ms | 24.395 ms |

Overrun event brackets were 18.536 and 20.534 ms, not exact loss times.
TX errors/interface drops stayed zero. Capture metadata reported no missing
timestamps/socket drops; offline metric parsing confirmed policy and joint
controller sources in both runs. All twelve encoder streams were verified near
50.03 Hz before each launch. Existing freshness/heartbeat checks passed.

Compact at 100 Hz feedback previously exceeded the two-overrun budget within
5.8–7.6 s; this 50 Hz run completed, but still had an overrun. This is encouraging,
not proof of causality or reliability: one short ordered pair, actual TX rates
differ, and the harness changed. Repeat balanced bounded trials with the same
buffered monitor before selecting a default, and evaluate older policy
observations from reduced feedback. Neither 50 Hz nor compact is promoted.

Both runs verified twelve IDLE on Rest, stopped runtime, restored and read back
all original 10 ms encoder periods; original xacro byte-compared. Final runtime
inactive/MainPID=0. No saved drive configuration, heartbeat, watchdog or policy
changes. Compact final IDLE verification Unix 1790605816.528576518.
Evidence: marvin-can-bounded.PbR3LR/feedback50-buffered-{idle,relative,compact},
live_metrics_buffered.py, test_buffered_monitor.py, run-feedback50-buffered.sh;
matching robot backups/can-bounded.BLOixM. Production code unchanged.

### September 28 near-75 Hz attempt: blocked by IMU readiness

Encoder interval is uint32 milliseconds, so exact 75 Hz (13.333 ms) is not
representable. Selected nearest 13 ms (~76.923 Hz), measured 76.967–76.984 Hz
across twelve motors while idle. Same buffered harness and unchanged limits.

Relative trial never activated: BNO055 logged "Data fusion not ready, resend
read request" then shut down at Unix 1790606081.462543409. Controller rejected
activation at 1790606097.815485621: "no fresh IMU sample". This is an IMU startup
failure, not evidence of a CAN feedback-rate failure. CAN TX stayed 37,867 and
RX errors/overruns 118 throughout harness; there was no active exposure or
compact trial. Do not report a successful near-75 Hz comparison.

Rest remained inactive; runtime stopped/MainPID=0. All twelve original 10 ms
encoder periods restored with readback (restore checks each motor IDLE first);
original xacro byte-compared. No saved configuration or guard changes. Investigate
IMU startup/readiness before retrying; do not bypass the fresh-IMU gate.
Evidence: marvin-can-bounded.PbR3LR/feedback13ms-buffered-relative, feedback-rate13.py,
verify-feedback13.py, run-feedback13-buffered.sh; matching robot backup directory.
Python compile/shell syntax and three buffered-monitor offline tests passed.

### September 28 authorized near-75 Hz retry completed

User requested one retry after the IMU startup failure. Same 13 ms encoder
setting (~76.98 Hz measured), buffered monitor and unchanged guards; IMU
readiness succeeded on both launches without bypass. Each trial completed 15 s.

| Mode | New RX overruns | Actual TX Hz/node | Mean / p95 / max echo span (ms) | Max monitor check interval |
| --- | --- | --- | --- | --- |
| Relative | 0 (119→119) | 41.085 | 22.366 / 30.915 / 68.483 | 24.245 ms |
| Compact | 2 (121→123) | 48.638 | 3.323 / 4.559 / 10.110 | 21.776 ms |

Compact events were isolated at +9.632 and +14.835 s, within the two-event
budget; no third event or burst occurred. TX errors/interface drops stayed zero.
Echo captures reported no socket drops/missing timestamps and both policy and
joint-controller metric sources were present. Software echo spans do not
measure exact motor application timing. These short ordered trials cannot rank
50 versus ~77 Hz reliably or establish causality; actual TX rates still differ.
No default promoted. Prior IMU failure remains an intermittent startup issue,
not fixed by the retry.

Both trials verified twelve IDLE, stopped runtime, restored/read back all twelve
original 10 ms encoder intervals, and byte-compared original xacro. Final
inactive/MainPID=0; final IDLE verification Unix 1790606389.603198362.
Evidence: marvin-can-bounded.PbR3LR/feedback13ms-retry-{relative,compact},
run-feedback13-retry.sh; matching robot backups/can-bounded.BLOixM.

### Buffer-capacity estimate versus an empirical operating limit

MCP2515 has two full receive buffers (RXB0/RXB1); rollover/filter configuration
affects their use. This capacity is fixed, not inferred from the test series.
For standard-ID Classical CAN with eight data bytes, Linux's frame-length
formula gives 111–135 bits including intermission and stuffing bounds, hence
111–135 microseconds at 1 Mbit/s. With two available receive slots and continuous
accepted frames, servicing is needed on a few-hundred-microsecond scale.
Existing occupancy, frame phase and SPI/interrupt latency change the available
headroom; this is NOT a measured maximum permitted scheduling delay.

Encoder-only offered traffic from twelve motors is 600 frames/s at 50 Hz,
~923 frames/s at 13 ms, or 1,200 frames/s at 100 Hz, plus heartbeats/other replies.
Average rates do not bound short bursts. Compact TX adds bus occupancy and
driver/SPI work, but its local TX echoes are not twelve incoming feedback frames.
An overrun counter increment is not necessarily the number of lost CAN frames.

Repeated trials can estimate empirical overrun incidence at these operating
points; they cannot locate an exact loss-free Hz cutoff, infer buffer occupancy,
or prove a hardware capacity change. To measure drain headroom directly, capture
CAN wire timing plus MCP2515 IRQ/SPI servicing (ideally an external analyzer),
without bypassing safety guards. Software reception timestamps are after driver
processing and cannot reconstruct the frames lost before reception.

Sources:
- https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/MCP2515-Family-Data-Sheet-DS20001801K.pdf
- https://raw.githubusercontent.com/torvalds/linux/master/include/linux/can/length.h

### September 28 alternating compact-feedback series

Completed authorized sequence: 20 ms #1, 13 ms #1, 20 ms #2, 13 ms #2,
20 ms #3, 13 ms #3. Each completed 15 s active with unchanged two-overrun/burst,
freshness, heartbeat and monitor guards. Period readback and passive rate checks
preceded each activation. Sender remained compact configured 50 Hz, zero gap.

| Encoder period | Approx feedback Hz | Overruns in trials 1/2/3 | Active exposure | Actual TX Hz/node |
| --- | --- | --- | --- | --- |
| 20 ms | 50 | 0 / 0 / 1 | 45 s | 48.63–49.02 |
| 13 ms | 76.98 | 0 / 0 / 2 | 45 s | 48.67–48.96 |

Counters are measured from preflight baseline through cleanup; the nonzero
increments occurred in active trials. Background/setup increases between trials
are excluded, not erased: e.g. 137→143 before 20 ms #2 and 144→148 before 13 ms #3.
Mean twelve-frame software echo spans: 3.259–3.302 ms at 20 ms feedback and
3.373–3.420 ms at 13 ms. No default promoted; one versus two events in 45 s each
cannot establish a statistically reliable ranking or a loss-free limit.

Policy summaries lack per-window timestamps here and include transition windows.
Some whole-capture joint-age maxima reach hundreds of ms in the final window.
Removing first/last nonempty windows gives header-age proxy means 6.22–6.92 ms
across these runs, with no clear separation. This trimming is not exact temporal
alignment, and joint message header age is not per-motor encoder sampling age.
Do not use these summaries to declare the physical observation-age tradeoff solved.

Buffer estimate: MCP2515 has two full receive buffers, not a configurable queue
of dozens of frames. With standard eight-byte CAN frames at 1 Mbit/s, approximate
wire spacing is 111–135 microseconds (including intermission/stuffing bounds).
Two free slots therefore provide only a few hundred microseconds of burst
headroom without servicing, depending on initial occupancy/phase/filter rollover.
This is an illustrative model, NOT a measured IRQ/SPI latency limit. At twelve
motors, encoder traffic alone averages 600 / ~923 / 1200 frames/s for 50 / ~77 /
100 Hz, but clustered arrivals matter more than averages. User-space 50 ms
diagnostic guard is unrelated to the kernel driver's sub-ms buffer-drain work.
Overrun increments do not necessarily equal lost-frame counts. Exact operating
headroom needs CAN-wire plus IRQ/SPI measurements; these trials cannot locate it.

Sources for buffer/frame model:
- https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/MCP2515-Family-Data-Sheet-DS20001801K.pdf
- https://raw.githubusercontent.com/torvalds/linux/master/include/linux/can/length.h

Each trial verified twelve IDLE and restored original 10 ms settings with readback.
Final service inactive/MainPID=0; original xacro byte-compared; final IDLE
verification Unix 1790607234.697880585. No drive save, watchdog or production code
change. Evidence local marvin-can-bounded.PbR3LR/series-{20,13}ms-{1,2,3}, script
run-feedback-series.sh and offline analyze-series.py; same robot backup root.
Next useful measurement is aligned per-motor encoder receipt→policy-consumption
age, or hardware IRQ/SPI capture for burst headroom—not more relaxed stop limits.

### September 28 repeat series including 100 Hz: stopped before 100 Hz

Planned three alternating compact trials each at 20/13/10 ms encoder feedback,
using the same buffered harness and fixed stop limits. First 50 Hz trial
completed 15 s with one overrun (163→164). First ~77 Hz trial aborted at
10.947 s on the third overrun (164→167), at +3.774, +10.537 and +10.947 s.
The final two increments were also within one second; total-budget check is
evaluated first, hence recorded reason "total overrun budget exceeded".

Series paused on that guard as announced. No 100 Hz trial or later repetitions
executed, no safety limits changed. Do not treat the interrupted pair as a
completed three-rate comparison or infer a loss-free cutoff. Both RX error and
overrun counters increased together; TX errors/interface drops stayed zero.

Rest verified twelve IDLE at Unix 1790608499.267199447, runtime inactive/MainPID=0.
All twelve original 10 ms encoder periods restored/read back; original xacro
byte-compared. Evidence marvin-can-bounded.PbR3LR/series2-{20,13}ms-1 and
run-feedback-series2.sh, feedback-rate10.py, verify-feedback10.py; matching robot
backup root. New 100 Hz helpers prepared/syntax-checked but not exercised.

### September 28 final authorized three-rate pass

User requested one last attempt. Announced one trial per rate, reordered to
100→50→~77 Hz to obtain the missing 100 Hz result first. Same compact sender,
buffered harness, fixed guards and 15 s target; no retries after final abort.

| Encoder feedback | Outcome | Overruns | Actual TX Hz/node | Mean / p95 echo span |
| --- | --- | --- | --- | --- |
| 100 Hz (10 ms) | Completed 15 s | 2 (173→175) | 48.642 | 3.512 / 5.137 ms |
| 50 Hz (20 ms) | Completed 15 s | 0 (177→177) | 48.861 | 3.169 / 4.109 ms |
| ~77 Hz (13 ms) | Aborted at 14.851 s | 3 (178→181) | 48.666 | 3.444 / 4.877 ms |

Feedback rates verified passively before each trial. TX errors/interface drops
stayed zero and echo capture metadata reported no drops/missing timestamps.
The final abort was total-overrun budget, not IMU readiness or monitor timing.
Background/setup increments are excluded. Do not conclude ~77 Hz is intrinsically
worse than 100 Hz from one short ordered pass; intermittent/burst timing remains
a confound. 50 Hz looks promising for receive load but remains nonzero-error in
earlier trials. No rate/mode promoted and no further physical tests scheduled.

Each trial restored original 10 ms feedback with all twelve readbacks; final
twelve IDLE verified Unix 1790608897.055380701. Runtime inactive/MainPID=0,
original xacro restored and byte-compared. Normal relative sending and 100 Hz
feedback remain the default. Evidence marvin-can-bounded.PbR3LR/final-{10,20,13}ms-1,
run-feedback-final.sh, matching robot backups/can-bounded.BLOixM.
