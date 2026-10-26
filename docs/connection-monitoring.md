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
