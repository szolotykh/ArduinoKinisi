# Arduino API v2

Call `begin(timeout_ms=1000, heartbeat_timeout_ms=500)` before commands. It starts Wire,
identifies the SDK as Arduino 2.5.0, reads the controller identity, and waits for
READY. `boardInfo()` exposes hardware revision, protocol version and firmware Git
build identity; `ready()` reports session readiness.

The library includes protocol 2.11 commands while accepting firmware 2.1+ within
protocol major 2. Major versions break existing wire interfaces and must match.
Minor versions add compatible commands; patch versions provide compatible fixes
and do not block connection. New commands are gated individually. Protocol
versions are independent of SDK releases and physical board revisions; INIT
does not translate between major versions.

The SDK advertises subscription support and **no wall-clock capability**. The board
uses uptime and skips time-sync exchanges, including periodic sync. Call `poll()`
frequently from `loop()` to service [heartbeat and subscriptions](connection-monitoring.md).
`get_time_status()` reports the selected mode and default 30,000-ms interval;
changing that interval does not enable wall-clock sync in an uptime session.

## Bundled board profiles and sensors

`const KinisiBoardProfile* profile = controller.boardProfile()` returns the
profile selected from the INIT board model and complete hardware revision, or
`nullptr` before successful setup and for unknown boards. A new `begin()` and
session failures clear it. Treat the pointer as a view of the current session;
copy the struct if a historical snapshot is needed. No other revision is used
as a fallback. Unknown boards can still use compatible low-level commands.

Profiles contain `model`, `major`, `minor`, `patch`, `resources` (motors,
encoders, GPIO and status LED counts), `motor_features`, temperature IDs and
IMU features. Use `resources.motors`, `temperature_count` and `imu_count` to
delimit their arrays. Each IMU has an ID and accelerometer/gyroscope flags.
Profiles omit pins, buses, addresses, control models and component values.
Per-motor `current_sense`, `current_limit`, and `fault_reporting` indicate fitted hardware.
`supportsMotorCurrent(index)` checks current-sensing hardware, protocol support,
session readiness and Wire capacity. Current reads and subscriptions require
protocol 2.7; see [motor current](connection-monitoring.md).
Programmable-current commands remain unimplemented in firmware.

The generated tables are read-only and stored in `PROGMEM` on AVR. Lookup copies
one selected profile into the controller; no heap allocation or JSON parsing is
needed. `tools/update-board-profiles.py` imports the public firmware JSON using
an explicit local path or branch, following the same workflow as command updates.
Custom runtime profiles are not supported.

```cpp
const KinisiBoardProfile* profile = controller.boardProfile();
if (profile) {
    Serial.println(profile->resources.motors);
}
if (controller.supportsSensor(KinisiSensor::TEMPERATURE)) {
    double celsius = controller.get_temperature(0);
    if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(celsius);
}
if (controller.supportsSensor(KinisiSensor::IMU)) {
    imu_sample sample = controller.get_imu(0);
    if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(sample.acceleration_z);
}
```

`profile->hasSensor(KinisiSensor::TEMPERATURE, index)` and
`profile->hasSensor(KinisiSensor::IMU, index)` check physical presence only.
`controller.supportsSensor(kind, index=0)` additionally checks session readiness,
firmware protocol and Wire buffer capacity. It returns false for unknown boards;
direct reads on those boards rely on firmware availability errors.

| Method | Required protocol | Reply frame | Result |
| --- | --- | --- | --- |
| `get_temperature(uint8_t sensor_index)` | 2.4+ | 12 bytes | Celsius `double` |
| `get_imu(uint8_t sensor_index)` | 2.5+ | 52 bytes | `imu_sample` |
| `get_motor_current(uint8_t motor_index)` | 2.7+ | 12 bytes | Amperes `double` |
| `get_motor_faults(uint8_t motor_index)` | 2.8+ | 14 bytes | `motor_fault_status` |

`imu_sample` has `acceleration_x`, `acceleration_y`, `acceleration_z` in m/s² and
`angular_velocity_x`, `angular_velocity_y`, `angular_velocity_z` in rad/s.
Acceleration includes gravity; readings use native sensor axes. Accelerometer
and gyroscope samples are independent, without calibration, orientation or fusion.
The wire format is six binary64 values even on AVR, where native `double`
rounds them to 32-bit precision.

Older firmware produces local `UNSUPPORTED_COMMAND`; known absent sensors or
indices produce local `SENSOR_NOT_AVAILABLE`. Insufficient Wire capacity produces
`FRAME_TOO_LARGE`. These checks happen before transmission and keep the session
usable. Sensor startup, absence or I/O failures reported by firmware use
`KinisiFailure::CONTROLLER` with `SAMPLE_NOT_AVAILABLE`, `SENSOR_NOT_AVAILABLE`
or `INTERNAL_ERROR`. Non-finite readings are rejected as `MALFORMED_RESPONSE`,
invalidating the session. Always inspect `lastError()` before using a getter result.

## Requests and failures

The frame is `[length:u8, command:u8, message_id:u16 LE, payload]`; length excludes
itself. IDs increase from 1 through 65535, then wrap to 1. A new `begin()` does not
reset the counter. One operation is in flight at a time; this client is not
thread-safe. Replies must match both command and ID. Setters wait for ACK and
return `bool`. Getters retain their scalar return types or return named structs;
check `lastError().failure == KinisiFailure::NONE` before using a getter result.
A failed getter returns a zero-initialized value, which is not a valid sample.

`lastError()` contains a local failure category, controller `KinisiErrorCode`,
failed command, and message ID. Controller errors leave the session available
unless INIT/readiness has been lost. Transport, malformed-frame, and timeout
failures invalidate it; call `begin()` to establish readiness again. A timeout
does not mean a command was never executed. Operations are never automatically
resent, and the failed operation's ID is not immediately reused.

## Timestamped odometry

- `get_encoder_odometry(index)` returns `encoder_odometry_sample`: `timestamp_us`,
  `clock_mode`, `clock_quality`, and `angle`.
- `get_platform_odometry()` returns `platform_odometry_sample`: the same clock
  fields, plus `x`, `y`, and `t`.

Timestamps belong to acquisition, not the GET request. In this SDK, clock mode is
0 (uptime); a reset restarts the controller's time origin. All wire doubles are
8-byte IEEE binary64. On AVR boards whose `double` is 32-bit, numeric values are
converted to/from native precision; the wire layout remains unchanged.

## Wire transaction limits

The controller expects a **whole frame per I2C transaction**. Splitting a frame
into multiple `Wire` transactions is unsupported by the firmware. The client
checks request and response sizes before transmitting, and returns
`KinisiFailure::FRAME_TOO_LARGE` if either exceeds the configured Wire capacity.

A default 32-byte AVR Wire buffer supports INIT, encoder odometry, and many basic
commands, including temperature. It cannot carry the 52-byte IMU reply,
the 38-byte platform-odometry reply or
the 61-byte motor-controller-state reply. A **64-byte TX and RX buffer** supports motor position PID. Platform position
PID initialization needs a **100-byte TX buffer** (a 128-byte Wire configuration
is suitable); the P-only platform setup needs 52 bytes. Use a core with sufficient buffers or configure its actual
buffers, then set `KINISI_WIRE_BUFFER_SIZE` consistently for the library build.
The macro alone does not enlarge Wire. The library detects `BUFFER_LENGTH` or
`I2C_BUFFER_LENGTH`, otherwise conservatively assumes 32 bytes.

Reads reserve enough bytes for the expected response or a six-byte ERROR frame,
whichever is larger. The length prefix determines the payload; trailing bus
padding is ignored. Use firmware with the I2C zero-padding update included with this SDK change.
Older firmware can stretch the bus indefinitely when a read exceeds a short
response. Physical I2C validation is still required on the target board/core;
the native Wire tests simulate padding and short replies.

The response timeout uses wrap-safe `millis()` subtraction. Where
`WIRE_HAS_TIMEOUT` is available, `begin()` enables Wire's 100-ms bus timeout and
reset-on-timeout support. Other cores must supply their own bounded Wire calls:
an application deadline cannot interrupt a core stuck inside `requestFrom()`.

## Validation

Run `python tests/run_tests.py` with a C++11 compiler (`CXX` can select one).
The native suite uses fake Arduino/Wire interfaces; it does not validate voltage
levels, actual I2C clock stretching, encoders, or motor movement.

To compile against the real Arduino Uno core, run
`pio run -d tests/avr -e uno`. This compiles AVR numeric conversions and flash profile lookup;
it does not run the sketch or flash a board.

Host tests cover profile import validation, deterministic generation, exact
board matching, firmware gates, sensor payloads/errors, session cleanup and
32/52/64/128-byte Wire capacities. Optional sibling firmware tests exercise the
production C INIT/READY and framing code.

## Position and velocity tuning

Connections accept protocol 2.1+. Position calls require READY and protocol
2.2+, or 2.3+ for full PID setup. Unsupported position calls return `false` with
`UNSUPPORTED_COMMAND` without sending a frame or closing the session.

`initialize_motor_controller` and `start_platform_controller` default their final
`integral_limit` argument to **100 PWM percentage points**. It bounds I alone;
total PWM stays capped at +/-100% with firmware anti-windup. `INVALID_ARGUMENT`
reports invalid velocity gains or an integral limit outside 0..100 before any
bus transaction. Firmware 2.3.1 uses direct PID output, so retune older gains.
Kp=1, Ki=1, Kd=0 are example starting values, not a tuned motor profile.

Initialize velocity before position. Motor position uses continuous radians;
platform targets use world-frame meters/meters/radians. Position integral limits
are speed contributions (rad/s or m/s). Reset changes the origin and clears the
target. Velocity overrides cancel position mode; stops and reinitialization can
require position setup again. See [the motor/platform example](../examples/position-control/position-control.ino).

## Updating generated files

Commands and public board profiles come from the firmware repository. From the
library root, regenerate both using the same local firmware checkout:

```bash
python tools/update-commands.py --schema ../kinisi-motor-controller-firmware/commands.json
python tools/update-board-profiles.py --profiles ../kinisi-motor-controller-firmware/include/board_profiles.json
```

Both scripts also accept `--branch <firmware-branch>` to download their source
from an explicit firmware branch, or `--check` with either source option to
detect stale files without writing. Keep commands and profiles on the same
firmware revision. Invalid inputs are rejected before replacing generated files.

The snapshots are `tools/commands.json` and `tools/board_profiles.json`.
Generated commands, types, and `kinisi_board_profiles.h/.cpp` ship with the
library. Profiles are compiled into C++ tables; applications do not download
or parse JSON at runtime. Run the [validation checks](#validation) after updates.

## Motor current limits

Current-limit configuration requires protocol 2.11 and fitted hardware. The public
profile supplies numeric `supported_thresholds_ma` and `can_disable`; options are
hardware metadata, while the support helper also checks firmware availability.
Unknown/unsupported motors have no options. Values use integer milliamps and SDKs
reject settings outside the known profile before sending. Firmware validates them
independently. No board-specific enum is required.

```cpp
const KinisiCurrentLimitOptions* options = controller.getMotorCurrentLimitOptions(0);
if (controller.supportsMotorCurrentLimit(0) && controller.initialize_motor(0, false)) {
    motor_current_limit applied = controller.set_motor_current_limit(0, true, 5400);
    if (controller.lastError().failure == KinisiFailure::NONE)
        Serial.println(applied.threshold_ma);
    // options->threshold_count delimits options->supported_thresholds_ma.
}
```

Both GET and SET require an initialized, awake motor. SET stops the motor and its
controller, verifies the applied setting, and leaves the motor stopped even on
I/O failure. Platform-owned motors reject SET; release ownership first. GET does
not change motor state. Settings are volatile: motor initialization reapplies
board defaults, and driver power loss requires reinitialization. Disabling current
regulation leaves overcurrent protection unchanged.

An unknown board profile does not advertise support; explicit requests with a
compatible protocol can still reach firmware. No custom profiles are loaded.
See [firmware current-limit behavior](https://github.com/szolotykh/kinisi-motor-controller-firmware/blob/main/docs/motor-current-limit.md)
for errors, persistence, register behavior, and wire sizes.


## Motor PWM frequency

PWM frequency is one setting for all motors. It requires protocol 2.13 and a ready
connection; motors need not be initialized. Supported enum values are `KHZ_10`,
`KHZ_20`, `KHZ_25`, `KHZ_50`, and `KHZ_100`, represented in hertz on the wire.
Use the selected board profile's `pwm_frequency.supported_hz` to populate choices.
The default is available as `pwm_frequency.default_hz`.

Applying a frequency stops every motor and motor/platform controller and leaves
them stopped. It preserves duty-cycle resolution, motor direction configuration,
and driver current-limit settings. Restart motion explicitly. The selected frequency
survives motor initialization and reconnects. At boot, v0.4 restores an explicitly
saved frequency when a valid setup exists. Otherwise the board uses its default.
GET reads the board's active frequency selection without changing outputs.
Unsupported values are rejected; they are never silently rounded.

```cpp
if (controller.supportsPwmFrequency()) {
    const KinisiPwmFrequencyOptions* options = controller.getPwmFrequencyOptions();
    uint32_t appliedHz = controller.set_pwm_frequency(KinisiPwmFrequency::KHZ_25);
    if (controller.lastError().failure == KinisiFailure::NONE) {
        Serial.println(appliedHz);
    }
}
```

`get_pwm_frequency()` returns the active frequency in hertz. Both methods return
zero on failure; check `lastError()`. Requests and replies fit the default 32-byte
Wire buffer. `frequency_count` delimits `options->supported_hz`.

Unknown boards have no advertised options; explicit commands on compatible firmware
still reach the board for validation. Older firmware is rejected locally before I/O.

## Save and restore the platform setup

On a ready v0.4 board with protocol 2.14 or later, `supportsConfigurationSave()`
reports whether configuration storage is available. Initialize the platform and
apply the desired encoder settings, directions and PWM frequency. Calling
`save_configuration()` returns true after the firmware acknowledges a durable write. On failure it returns false and `lastError()` gives the reason.

After restart, connect and read the running setup before sending another
platform initialization. `get_configuration_field(field, index)` reads the
platform type (0), wheel diameter (1), track or length (2), width or radius
(3), encoder resolution (4), encoder direction (5), or motor direction (6).
Use index zero for platform fields and the channel index for encoder and
motor fields. `get_pwm_frequency()` reads the applied frequency in hertz.
The board restores the saved setup with outputs stopped. Reconnecting during
the same boot preserves running edits. Call Save again to retain those edits
across the next restart.

## Motor-driver sleep

`supportsDriverSleep()` requires READY, protocol major version 2, and an exact
known board profile with `power.driver_sleep` enabled. Original v0.3.0 and Nano
(hardware v0.3.1) require protocol 2.17 or later; v0.4.0 requires 2.15 or later.
Both commands reject unsupported sessions before I/O, including unknown patch
revisions. Nano's product name does not change its wire identity: model 1, 0.3.1.

```cpp
if (controller.supportsDriverSleep()) {
    if (controller.set_driver_sleep(1)) {  // Board ACK received.
        uint8_t sleeping = controller.get_driver_sleep();
        if (controller.lastError().failure == KinisiFailure::NONE) {
            Serial.println(sleeping);
        }
    }
    bool released = controller.set_driver_sleep(0);  // Wait for release ACK.
    if (!released) { /* Inspect controller.lastError() before retrying. */ }
}
```

The setter accepts uint8_t values 0 or 1 and returns false on failure. The
getter returns 0 or 1, and also returns zero on failure: always check
`lastError()` before treating zero as a released inhibit. Requests and responses
fit the default 32-byte Wire buffer.

Sleep stops and coasts all motors, cancels active control, and inhibits new
drive requests while communication remains available. The getter reads the
board-wide inhibit shared by all clients; zero does not imply that every
individual motor is initialized. Wake leaves outputs stopped. Initialize or
start closed-loop control again before issuing new targets. Prohibited
operations report `KinisiErrorCode::DRIVERS_ASLEEP` (19) in `lastError()`.
A failed wake leaves the inhibit set; correct the cause and explicitly retry.
A stale wake superseded by a newer sleep reports `DRIVERS_ASLEEP`; read the
current inhibit and explicitly retry only if release is still wanted.

On v0.3/Nano, sleep holds both DRV8870 inputs low. Releasing the software
inhibit keeps those inputs low until a fresh nonzero drive request or deliberate
brake, so a zero readback does not establish that the chips are physically awake.
After confirmed inhibit release, a fresh `brake_motor()` or `brake_platform()`
request is valid without a preceding drive request, subject to the command's
normal initialization requirements. Ordinary short brake drives both inputs
high; stop/coast leaves both low. Brake requests made while inhibited, or stale
requests from before the inhibit transition, are rejected.
These boards have no SPI wake verification, current telemetry or fault readback.
Their separate board-idle API requires protocol 2.18 or later.
On v0.4, wake verifies retained settings for initialized drivers.

Sleep is volatile and is not saved to FRAM. Reconnecting preserves its current
state and sends no automatic sleep or wake command. Reset clears the inhibit
and follows the board's normal boot and saved-configuration policy.

## Communication-preserving board idle

`supportsBoardIdle()` requires READY, protocol major version 2 and the exact
board profile's `power.board_idle` capability. Original v0.3.0 and Nano v0.3.1
require protocol 2.18 or later; v0.4.0 requires 2.16 or later. Older firmware
and unknown revisions fail locally before I/O. Driver-only sleep remains
available on v0.3/Nano at protocol 2.17 and v0.4 at protocol 2.15.

```cpp
if (controller.supportsBoardIdle()) {
    if (controller.set_board_idle(1)) {  // Board ACK received.
        uint8_t idle = controller.get_board_idle();
        if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(idle);
    }
    bool awake = controller.set_board_idle(0);  // Explicit wake, outputs stopped.
    if (!awake) { /* Inspect lastError(), correct the cause, then retry. */ }
}
```

The setter accepts uint8_t values 0 or 1 and returns false on failure. The
getter returns 0 or 1, or zero on failure: check `lastError()` before interpreting
zero as awake. Both commands fit the default 32-byte Wire buffer.

This is shallow idle with communication running, not MCU STOP or STANDBY.
Entry stops control, inhibits the drivers, and pauses existing subscriptions.
Heartbeat, time sync and supported direct reads remain available. New
subscriptions and independent driver-sleep changes report `KinisiErrorCode::BOARD_IDLE`
(20) through `lastError()`. Motion requests report `DRIVERS_ASLEEP` (19).
The I2C SDK session stays connected; the board also keeps its USB session
available for ordinary enter/read/wake requests. No reconnect or initialization
is needed for an idle transition. Keep calling
`poll()` to service communication while idle.

Wake preserves an independent driver-sleep request that was active on entry.
Otherwise it clears the idle-imposed inhibit and leaves outputs stopped.
v0.4 verifies initialized driver settings over SPI. On v0.3/Nano, both DRV8870
inputs remain low until a fresh nonzero drive or explicit short-brake request;
successful wake acknowledges the software state, not physical chip wake.
Read `get_driver_sleep()` separately; it reports the effective inhibit, including
board idle, and cannot reveal the prior independent sleep request while idle.
Waking the board does not necessarily wake its drivers. A failed transition keeps idle and motion inhibition active
for an explicit retry. The SDK sends no extra driver, initialization,
controller-start, or motion commands around idle transitions. Reconnect does
not wake the board. Idle is volatile and clears on reset.

`getSubscriptionSample()`, `getSubscribedEncoderOdometry()` and
`getSubscribedPlatformOdometry()` retain the last sample and its original
`timestamp_us` through idle and wake. A true return value means cached data is
available; it does not certify freshness. Frames buffered before entry may
still arrive. Track sample timestamps in their clock domain and wait for a new
post-wake sample before using streamed data as current. Existing streams resume
without resubscribing; a direct read does not refresh the subscription cache.

## Encoder and IMU experiment

Exact v0.4.0 boards with protocol 2.19 can opt into the volatile comparison
experiment. Legacy encoder control remains the default. See [mounting, health,
compact diagnostics and examples](fusion-experiment.md).

## IMU-only relative tracking

See [IMU-only relative tracking](imu-tracking.md) for the protocol 2.20 opt-in,
stationary sensor-axis mounting declaration, bounded horizon and diagnostics.

## Tracking source selector

A ready model 1 board with hardware 0.4.0 and protocol 2.21+ can report encoder,
IMU-only or passive fused tracking. Encoder is the default. Call
`set_feedback_source(source)` only while stopped and disarmed, with the motors
coasting and a quiet encoder observation. `SOURCE_SWITCH_UNSAFE` leaves the previous
preference in place. Selection never starts a producer, commands motion or enables
a control loop. Prepare the required producer explicitly.

- Encoder and passive fused tracking require initialized platform odometry.
- The passive observer additionally needs an explicit `set_feedback_fusion_mount(yaw)`
  declaration for a rigid, centered, level board with +Z up. Yaw is the angle of
  board +X in chassis XY, counterclockwise about chassis +Z, in radians. Read
  `get_feedback_fusion_mount()` to confirm it. Mount declarations are volatile.
- IMU-only tracking requires the stationary mounting declaration and cold calibration
  described above. Its one-second diagnostic window begins at the producer origin.
  Re-selecting a source does not extend the window.

The board reports `requested`, `active`, `valid`, `reason` and the native
`source_detail`. Loss or expiry gives active NONE. Discard old estimates on source
loss, a change in `origin_id`, or reconnect. Pose uses metres and continuous radians
relative to the selector origin. Twist uses chassis forward/left metres per second
and counterclockwise radians per second. The producer supplies `sequence` and the
board-monotonic acquisition `timestamp_us`. `age_us` is the board's measured sample
age, with `UINT32_MAX` for unavailable. Read current status after reconnect.

Only explicit `save_configuration()` persists the requested preference with the
rest of the running setup. Reboot requires a fresh mount or calibration as applicable.
Restored preference is observable in selector status after reboot. Tracking data,
origin and controller targets are volatile.

Call `controller.readFeedbackSourceSnapshot(status, pose, twist)` and branch on
`KinisiFeedbackRead`. `VALID` supplies a matched sample. `UNAVAILABLE` supplies a
board-reported invalid status. `CHANGED` means three successive reads could not
establish a common sample. `ERROR` preserves the transport or controller failure in
`lastError()`. Vectors are cleared on every result other than `VALID`. A failed write
requires explicit readback before deciding whether to retry.
