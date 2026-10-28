# Heartbeat and subscriptions

`begin()` uses controller uptime, waits for INIT/READY, and enables a 500 ms watchdog. Call `poll()` frequently in `loop()` so the library can send a heartbeat after 100 ms without other outgoing traffic. Avoid long blocking delays. A stalled sketch stops servicing the watchdog and firmware stops the motors.

`begin(1000, 0)` disables automatic monitoring at setup; the first argument remains the request timeout. Monitoring can be changed with `set_heartbeat_config(enabled, timeout_ms)` and read with `get_heartbeat_config()`. The heartbeat idle interval is one fifth of the configured timeout. Disabling monitoring also clears subscriptions.

Call `subscribe(KinisiTopic::TEMPERATURE, 0, 250)` to stream temperature, then use `getSubscriptionSample(topic, index, sample)` to copy the latest value. `unsubscribe(topic, index)` stops delivery without stopping calculation. Existing GET methods still work. Odometry must be initialized and running before subscribing.

```cpp
// After a successful begin():
if (controller.supportsTopic(KinisiTopic::TEMPERATURE))
    controller.subscribe(KinisiTopic::TEMPERATURE, 0, 250);

// Inside loop():
if (controller.poll()) {
    KinisiTelemetrySample sample;
    if (controller.getSubscriptionSample(KinisiTopic::TEMPERATURE, 0, sample))
        Serial.println(sample.data.temperature.celsius);
}
```

Generic subscriptions require protocol 2.6 or newer within major 2. `supportsTopic(topic, index=0)` checks protocol, known hardware, and Wire capacity. Unknown profiles return false; an explicit request lets firmware validate hardware presence. An unsupported older protocol is rejected before sending.

| `KinisiTopic` | Union member in `sample.data` | Full event size | Minimum interval |
| --- | --- | --- | --- |
| `ENCODER_ODOMETRY` | `encoder_odometry` | 24 bytes | Twice calculation period |
| `PLATFORM_ODOMETRY` | `platform_odometry` | 40 bytes | Twice calculation period |
| `TEMPERATURE` | `temperature` | 24 bytes | 250 ms |
| `IMU` | `imu` | 64 bytes | 10 ms |
| `MOTOR_CURRENT` | `motor_current` (`amps`) | 24 bytes | 20 ms |

Motor current uses motor indices 0–3 and requires protocol 2.7 and fitted sensing
hardware. `supportsMotorCurrent(index)` checks pull support. The 12-byte pull
response and 24-byte event both fit a default Uno Wire buffer.

```cpp
// After begin(), in setup():
if (controller.supportsMotorCurrent(0)) {
    double amps = controller.get_motor_current(0);
    if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(amps);
    controller.subscribe(KinisiTopic::MOTOR_CURRENT, 0, 100);
}
// In loop():
controller.poll();
KinisiTelemetrySample sample;
if (controller.getSubscriptionSample(KinisiTopic::MOTOR_CURRENT, 0, sample))
    Serial.println(sample.data.motor_current.amps);
// When finished: controller.unsubscribe(KinisiTopic::MOTOR_CURRENT, 0);
```

Values are nonnegative filtered high-side current estimates. PWM affects the
measurement; it is not signed winding or battery current. Reading does not enable
the motor. Check `lastError()` after a pull; a zero return can represent failure.
ADC saturation reports `SAMPLE_OUT_OF_RANGE`. Failed streamed reads pause delivery
until recovery instead of repeating an old sample.

The enum selects the valid union member. Platform index is 0; other topics use their resource index. Each subscription stores one latest sample. The default `KINISI_MAX_SUBSCRIPTIONS` is 8; set it consistently for the library and sketch to a value from 1 to 16 to trade RAM for capacity. The client returns `KinisiFailure::SUBSCRIPTION_LIMIT` before sending when local storage is full. Firmware can also return its own `SUBSCRIPTION_LIMIT` error. Replacing a subscription clears its old cached sample; a rejected replacement preserves it.

`poll()` services the I2C bus for active subscriptions as well as idle heartbeats. I2C requires master-initiated transfers: the SDK uses `POLL_TELEMETRY` to receive a due event followed by an ACK, or just an ACK when no sample exists. It also accepts events interleaved with ordinary responses. Keep calling `poll()` at least as often as the subscription interval divided by the number of active subscriptions. Firmware owns the requested cadence; slow bus service skips intermediate measurements.

`getSubscriptionSample()` returns false before the first sample and performs no bus traffic. Timestamps retain integer microseconds and clock metadata. Odometry timestamps mark acquisition; sensor timestamps mark register-read completion. There is no time-sync exchange for Arduino clients.

An enabled watchdog is required. With the default 50 ms calculation period, odometry accepts intervals of 100 ms or longer. The actual Wire implementation and `KINISI_WIRE_BUFFER_SIZE` must both fit the full event; oversized subscriptions are rejected before transmission. An IMU GET fits in 52 bytes, but its timestamped subscription event needs 64 bytes.

`subscribe_odometry(source, interval_ms)` and `unsubscribe_odometry(source)` are retired since protocol 2.6, remain supported throughout major 2, and are eligible for removal in major 3. Their existing cache accessors are `getSubscribedEncoderOdometry()` and `getSubscribedPlatformOdometry()`. Legacy encoder events use 23 bytes and platform events use 38 bytes. Switching between generic and legacy subscriptions replaces the same odometry source.

On watchdog timeout or USB loss, firmware stops motors and requires fresh INIT before further operation. A subsequent `begin()` does not restart motion or recreate subscriptions. Inspect `lastError()` when a command or `poll()` returns false.

## Motor faults

Fault reporting requires protocol 2.8 and fitted hardware. Read `status_valid`
before interpreting diagnostic flags; an asleep or unavailable driver must not
be shown as healthy. Reads do not wake the driver, initialize the motor, clear
latched faults, or change the outputs. Missing motor power cannot be distinguished
from communication or driver failure by an unavailable response alone. Existing
motor reinitialization clears latched driver faults. Fault subscriptions have a
100 ms minimum interval and report snapshots, not a fault-event history.

```cpp
// After begin(), with poll() serviced from loop():
if (controller.supportsMotorFaults(0)) {
    motor_fault_status status = controller.get_motor_faults(0);
    if (controller.lastError().failure == KinisiFailure::NONE && status.status_valid)
        Serial.println(status.overcurrent);
    controller.subscribe(KinisiTopic::MOTOR_FAULTS, 0, 250);
}
// In loop(), after a successful poll():
KinisiTelemetrySample sample;
if (controller.getSubscriptionSample(KinisiTopic::MOTOR_FAULTS, 0, sample) &&
        sample.data.motor_faults.status_valid)
    Serial.println(sample.data.motor_faults.overcurrent);
// When finished: controller.unsubscribe(KinisiTopic::MOTOR_FAULTS, 0);
```

Fault reads use a 14-byte frame and subscription events use 26 bytes, both within
the default Uno Wire buffer.

See the [firmware fault reference](https://github.com/szolotykh/kinisi-motor-controller-firmware/blob/main/docs/motor-faults.md) for every diagnostic field and hardware limitations.


## Subscription delivery modes

`PERIODIC` (0) sends the latest sample at each interval. `ON_CHANGE` (1)
sends an initial snapshot, then another when the encoded value fields change.
Timestamp and clock metadata are excluded from the comparison. Fault appearance,
clearance, sleep, and diagnostic validity changes are included in the value fields.
Numeric measurements use exact encoded comparisons, with no deadband.

The interval controls how often firmware checks for changes; topic minimums still
apply. Faults can be checked every 100 ms. This is sampled detection, not an
interrupt-driven guarantee. A quiet change subscription remains active, and the
connection heartbeat runs independently. Hardware protection remains independent
of telemetry delivery.

Each connection keeps its own baseline. Replacing a subscription requests a fresh
initial snapshot. Failed reads pause delivery; the next successful read sends a
snapshot even if the value matches the previous one. A failed transmit does not
advance the baseline. Slow transports receive the latest state and can skip
intermediate transitions; subscriptions are not a fault history. On I2C, the
master must continue servicing `poll()` to retrieve queued samples.

Delivery modes require protocol 2.9. The SDK keeps the original `SUBSCRIBE`
request for periodic mode, so existing applications and older compatible firmware
keep working. Other modes use `SUBSCRIBE_WITH_MODE` (0x83), with the same topic,
index and uint32 interval followed by a uint8 mode. Events and unsubscribe keep
their existing formats. Unsupported modes are rejected before transmission.

```cpp
controller.subscribe(KinisiTopic::MOTOR_FAULTS, 0, 100,
                     KinisiSubscriptionMode::ON_CHANGE);
// In loop():
controller.poll();
KinisiTelemetrySample sample;
if (controller.getSubscriptionSample(KinisiTopic::MOTOR_FAULTS, 0, sample)) {
    // Cached state remains available while the driver state is unchanged.
    bool valid = sample.data.motor_faults.status_valid;
    bool overcurrent = sample.data.motor_faults.overcurrent;
    // Use valid and overcurrent in the application.
}
```

The mode-aware request occupies 11 bytes and a motor-fault event occupies 26 bytes;
both fit the default 32-byte Wire buffer. `getSubscriptionSample()` returns cached
state, not an indication that a new event arrived; compare timestamps if needed.


## Clearing motor faults

`clear_motor_faults(motor_index)` requires protocol 2.10 and returns fresh
motor-fault status. Firmware stops the motor and its active controller, clears
latched flags, and leaves the output disabled with zero PWM. Read and record
diagnostics first when investigating an issue. Existing active conditions may
still be reported after the clear; inspect the response before restarting.

The motor must already be initialized and its driver awake. Platform-owned motors
are rejected with `MOTOR_OWNED`; initialization is checked next. SPI or verification
failures return `INTERNAL_ERROR` and leave the motor stopped once execution has
begun. No automatic initialization, wake-up, retry, or motor restart is performed.
The existing per-motor `fault_reporting` profile flag identifies the supported
driver hardware, while protocol 2.10 identifies command availability.

See [firmware fault reporting](https://github.com/szolotykh/kinisi-motor-controller-firmware/blob/main/docs/motor-faults.md)
for register behavior and the meaning of each returned flag.

```cpp
motor_fault_status status = controller.clear_motor_faults(0);
if (controller.lastError().failure == KinisiFailure::NONE) {
    Serial.println(status.fault_active);
}
```

Always check `lastError()` before using the returned object. The 5-byte request
and 14-byte response fit the default 32-byte Wire buffer.

## Trigger and delivery settings

`SUBSCRIBE_CONFIGURED` (protocol 2.12) separates when to read from what to send.
It sends an initial snapshot, then applies these settings:

| Setting | Values | Meaning |
| --- | --- | --- |
| `trigger` | `PERIODIC` (0), `ON_EVENT` (1) | Read on the polling schedule or on a source notification. |
| `delivery` | `ALL` (0), `ON_CHANGE` (1) | Send every triggered update, or only changed value fields. |
| `poll_interval_ms` | Integer milliseconds | Required for periodic triggering; zero for event triggering. Topic minimums apply. |
| `max_silence_ms` | Integer milliseconds | For ON_CHANGE, send a fresh snapshot after this long without a successful send, even if unchanged. Zero disables refresh. |

Nonzero intervals must meet the topic minimum and cannot exceed 3,600,000 ms.
ALL requires zero maximum silence. Timestamp and clock metadata do not count as
value changes. Odometry retains its acquisition timestamp; refresh does not start
or restart calculation. Heartbeat monitoring remains independent and required.

Only MOTOR_FAULTS currently supports ON_EVENT. The fault GPIO interrupt records a
notification; the command task reads diagnostics and sends them as soon as it can.
A 100 ms background check catches additional diagnostic changes while the fault
pin is already asserted, and recovery conditions without an interrupt. Driver
initialization enables thermal-warning reporting on the fault pin while preserving
protection settings. Current-regulation reporting on that pin remains unchanged.

ON_EVENT + ALL sends each observed notification batch, including unchanged values,
and changes found by reconciliation. An unchanged background check is not an event.
ON_EVENT + ON_CHANGE filters repeated values and can refresh after maximum silence.
The firmware stores latest state, not a lossless event queue: bursts can coalesce,
transient conditions may be missed, and bus/task delays affect delivery. Failed
reads do not resend old data. Backpressure does not acknowledge a change until sent.
Maximum silence is a best-effort reporting deadline, not a connection watchdog or
a guarantee during failed reads, unavailable measurements, or a blocked transport.

The request payload is `uint8 topic_id`, `uint8 index`, `uint32 poll_interval_ms`,
`uint8 trigger`, `uint8 delivery`, `uint32 max_silence_ms`, in that order. Events,
unsubscribe, session cleanup and capacity limits remain shared with the existing
subscription commands. Existing interval/mode APIs retain their wire behavior.
Configured subscriptions require 2.12 and are rejected locally on older firmware.

On I2C, the board cannot initiate transfers. Continue calling Arduino `poll()`;
event subscriptions request service every 10 ms (divided among active streams).
USB receives notifications without client polling. Hardware protection acts
independently of subscription and transport timing.

```cpp
bool started = controller.subscribe(
    KinisiTopic::MOTOR_FAULTS, 0, 0,
    KinisiSubscriptionTrigger::ON_EVENT,
    KinisiSubscriptionDelivery::ON_CHANGE, 5000);
```

Check the returned bool and `lastError()`. Continue servicing `controller.poll()`
in `loop()` and use `getSubscriptionSample(KinisiTopic::MOTOR_FAULTS, 0, sample)`.
A fault event is 26 bytes and the configured request is 16 bytes; both fit a
32-byte Wire buffer. Other topics retain existing frame-size restrictions.
The explicit `subscribe_configured(...)` method has the same settings and validation.
