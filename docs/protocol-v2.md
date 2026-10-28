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
