# Arduino API v2

Call `begin(timeout_ms=1000, heartbeat_timeout_ms=500)` before commands. It starts Wire,
identifies the SDK as Arduino 2.1.0, reads the controller identity, and waits for
READY. `boardInfo()` exposes hardware revision, protocol version and firmware Git
build identity; `ready()` reports session readiness.

The SDK advertises subscription support and **no wall-clock capability**. The board
uses uptime and skips time-sync exchanges, including periodic sync. Call `poll()`
frequently from `loop()` to service [heartbeat and subscriptions](connection-monitoring.md).
`get_time_status()` reports the selected mode and default 30,000-ms interval;
changing that interval does not enable wall-clock sync in an uptime session.

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
commands. It cannot carry, for example, the 38-byte platform-odometry reply or
the 61-byte motor-controller-state reply. A **64-byte TX and RX buffer** supports
all current commands. Use a core with sufficient buffers or configure its actual
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
`pio run -d tests/avr -e uno`. This compiles the AVR numeric-conversion path;
it does not run the sketch or flash a board.
