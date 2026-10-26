# ArduinoKinisi

Arduino library for controlling Kinisi motion controllers over I2C. Use it to
control motors and platforms, read encoders and sensors, and subscribe to live
measurements. Bundled board profiles let applications check which features the
connected controller supports.

## Getting started

Follow the [Arduino library installation guide](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries)
to install this library manually. Connect the controller's I2C interface and
common ground to your Arduino; the default controller address is `8`.

This library targets the supported 5 V Arduino setup; 3.3 V-only Arduino boards
are not supported. Firmware must use a compatible protocol major version.
Some commands and subscriptions require larger Wire buffers than the default;
check the [protocol and buffer requirements](docs/protocol-v2.md) for your application.

This example toggles the controller's status LED once per second. Keep calling
`poll()` in `loop()` to service the connection and any active subscriptions.
If setup, polling, or a command fails, inspect `lastError()`; see
[connection handling](docs/connection-monitoring.md) for recovery.

```cpp
#include <kinisi.h>

KinisiController controller(8);

void setup() {
  if (!controller.begin()) {
    while (true) delay(1000);
  }
}

void loop() {
  if (!controller.poll()) return;
  static uint32_t toggled = 0;
  if (uint32_t(millis() - toggled) < 1000) return;
  toggled = millis();
  if (!controller.toggle_status_led_state()) return;
}
```

## Documentation and examples

- [Protocol, board profiles, errors, and Wire requirements](docs/protocol-v2.md)
- [Connection monitoring and subscriptions](docs/connection-monitoring.md)
- [Example sketches](examples)
- [Updating generated commands and board profiles](docs/protocol-v2.md#updating-generated-files)
- [Controller command reference](https://github.com/szolotykh/kinisi-motor-controller-firmware/blob/main/commands.md)

## Related projects

- [Kinisi firmware](https://github.com/szolotykh/kinisi-motor-controller-firmware)
- [Kinisi hardware](https://github.com/szolotykh/kinisi-motor-controller-board)
- [JavaScript SDK and web client](https://github.com/vsreality/jskinisi)
- [Python SDK](https://github.com/szolotykh/pykinisi)
