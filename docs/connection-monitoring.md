# Heartbeat and odometry subscriptions

SDK 2.1 requires protocol 2.1 firmware. `begin()` uses controller uptime, waits for INIT/READY, and enables a 500 ms watchdog. Call `poll()` frequently in `loop()` so the library can send a heartbeat after 100 ms without other outgoing traffic. Avoid long blocking delays. A stalled sketch stops servicing the watchdog and firmware stops the motors.

`begin(1000, 0)` disables automatic monitoring at setup; the first argument remains the request timeout. Monitoring can be changed with `set_heartbeat_config(enabled, timeout_ms)` and read with `get_heartbeat_config()`. The heartbeat idle interval is one fifth of the configured timeout. Disabling monitoring also clears subscriptions.

For streaming, initialize the encoder and start odometry, then call `subscribe_odometry(encoder_index, interval_ms)`. Sources 0–3 select encoders and source 4 selects platform odometry. Calculation remains independent; `unsubscribe_odometry(source)` stops messages without stopping calculation. Existing GET methods still work.

`poll()` services the I2C bus for active subscriptions as well as idle heartbeats. I2C requires master-initiated transfers: the SDK uses `POLL_TELEMETRY` to receive a due event followed by an ACK, or just an ACK when no sample exists. It also accepts events interleaved with ordinary responses. Keep calling `poll()` at least as often as the subscription interval divided by the number of active subscriptions. Firmware owns the requested cadence; slow bus service skips intermediate measurements.

Use `getSubscribedEncoderOdometry(index, sample)` or `getSubscribedPlatformOdometry(sample)` to copy the latest cached event. These return false before the first event and do not access the bus. Timestamps retain the actual calculation time in controller uptime microseconds. There is no time-sync exchange for Arduino clients.

The subscription interval must be at least twice the calculation period: default 50 ms calculations allow 100 ms or longer subscriptions. An enabled watchdog is required. Encoder events fit a 32-byte Wire buffer; platform events require at least 38 bytes and are rejected locally on a 32-byte configuration. A 64-byte Wire implementation supports both.

On watchdog timeout or USB loss, firmware stops motors and requires fresh INIT before further operation. A subsequent `begin()` does not restart motion or recreate subscriptions. Inspect `lastError()` when a command or `poll()` returns false.
