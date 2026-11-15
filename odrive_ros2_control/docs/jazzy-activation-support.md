# Jazzy activation support

Issue: [marvin_wiki#59](https://github.com/Trana/marvin_wiki/issues/59).
Supported target: Ubuntu24.04 / ROS2 Jazzy. Source implementation and physical acceptance are separate.

Jazzy requires active hardware before a command controller can claim movement interfaces.
The old `require_command_receipt` activation guard required a position claim first.
Its combined encoder/claim error made fresh encoder feedback appear stale.
Removing that guard alone would leave the sender without a preclaim position source.

## Ownership

The driver owns a measured-position source until the native controller supplies a complete valid command.
It validates every encoder's finite position and kernel receipt within100ms before any motor request.
It checks all position-mode sends, then all measured position primes, before any CLOSED_LOOP request.
Claims remain real ROS claims. Measured targets never overwrite native command or receipt storage.

`PositionSupport` owns the source and retained target. `PacedCommands` remains the only sender schedule.
The existing transport mutex serializes lifecycle writes, feedback and sender writes.
Bootstrap snapshots have their own driver generation. They do not renew native command receipts.
The sender refreshes them only between coherent rounds. The existing100ms encoder bound applies to every hold round.
Existing batch deadlines and optional reviewed fallback limits still apply. The fallback default stays disabled.
There is no second CAN writer or scheduler.

The first native snapshot requires position claims for every axis, finite positions and enabled feedforward,
and real host-monotonic receipts after activation, no later than now and younger than `can_command_stale_ms`.
Repeated hardware writes cannot renew an old receipt. A velocity-only or effort-only takeover is rejected before any claim mutation.
The generic non-receipt path retains its existing velocity/effort modes.

Until native admission, healthy measured hold can continue without claims or hardware `write()` callbacks.
Initial IDLE heartbeats do not establish support. CLOSED_LOOP acknowledgement acquisition uses the reviewed observation-age limit.
When the reviewed fallback is enabled, hold after that interval requires current error-free motor evidence, encoder feedback, tracking and submissions.
Without that policy, the driver enforces encoder freshness but does not report observed physical support.
Losing those checks makes support unconfirmed and inhibits output. No software can guarantee torque after transport or drive failure.
ODrive watchdog configuration is unchanged.

After native admission, a missing/stale/invalid receipt or claim loss retires that producer.
The driver retains the last complete submitted position-only round, including a partial native round that completes within its existing deadline.
It never returns to the initial activation pose after movement.
The exported support state becomes recovery hold, which makes the existing supervisor require explicit recovery.
A later native message cannot resume the retired producer. Verified Stop and a new activation are required.

## Partial activation and Stop

A failed mode or prime rejects activation before CLOSED_LOOP.
Before the first possible CLOSED_LOOP request, the driver installs the hold source and activation epoch.
A failed enable returns `CallbackReturn::ERROR`. Some drives can already be energized.
`on_error()` preserves the session, transport and sender without IDLE. Jazzy enters UNCONFIGURED.
The existing supervisor retains UNKNOWN and refuses another activation from that state.

Jazzy skips ordinary hardware read/write callbacks while UNCONFIGURED.
The retained sender therefore runs the same bounded receive path under the same mutex during error recovery.
The receive budget remains256 frames or1ms, checked after each frame.
Receive timestamps remain real kernel timestamps. This does not create native command freshness.

Explicit Stop requests INACTIVE through the existing hardware-state service.
From UNCONFIGURED, Jazzy invokes configure. Recovery configure first requests and verifies fresh IDLE
through the retained transport, before any reset or reinitialization.
A missing acknowledgement leaves UNCONFIGURED and Stop incomplete. A later Stop retries verification.
Only successful verification retires the session and reports INACTIVE.
This avoids the old failure case where activation FAILURE leaves INACTIVE and repeated INACTIVE becomes a lifecycle no-op.

`StopAdmission` admits a completed Stop's successor by generation.
A Stop already waiting for the transport mutex, or one concurrent with admission, cannot be erased by a Boolean reset.
Activation checks this token before every mode, prime and enable send. Stop also fences retained and native output.

Sender software faults no longer request automatic IDLE. This deliberately changes legacy driver behavior
to match the November15 supervisor contract. Explicit Rest/shutdown/disable remains the torque-removal authority.
Successful explicit Stop still requires fresh acknowledgements from every configured drive.
Historical measurements of the previous fault behavior remain historical.

## Offline verification

These checks use fake CAN and do not open SocketCAN. They cannot establish physical support or balance.

1. Build `odrive_ros2_control` with `BUILD_TESTING=ON` in an isolated build/install directory.
2. Run CTest in that package's build directory.
3. Run `test/test_jazzy_manager.py` with localhost-only domain226 and the selected Marvin controller source on `PYTHONPATH`.
4. Set `ODRIVE_JAZZY_FAKE_LIBRARY` to the built `libodrive_jazzy_fake_transport.so`.

The manager test creates a private plugin resource and controller-manager process under its temporary directory.
It uses the actual `HardwareStateManager` activation and Stop helpers.
It tests partial enable ERROR→UNCONFIGURED, refusal of activation retry, no automatic IDLE,
missing Stop acknowledgements, successful Stop retry, and normal hardware-before-controller activation.
The test-only library is not installed or exported by the production package.

`test_jazzy_activation_driver` additionally tests12 axes, delayed/partial claims, absent write callbacks,
mode/prime failures, finite/future/missing receipts, native transfer and retained-target recovery.
Metrics and timing-off variants run. `test_position_support` checks complete-round retention and concurrent Stop generation admission.
Existing encoder, pacing, coherent batch, fallback and verified-IDLE tests remain required.

Before physical activation, build and run these checks on the actual Pi's installed Jazzy version.
Admit the selected library through the canonical stopped-only profile transaction.
Then repeat physical inactive Start/Stop before supported-stand IK, policy and loaded timing checks.
Do not interpret a fake-transport pass as permission for an uncoordinated hardware test.
