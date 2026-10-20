# ArduinoKinisi
Arduino library for kinisi motion controller. This library allows to control kinisi motion controller from Arduino board via I2C interface.\
Discription of the motor controller commands can be find here: [Kinisi Motion Controller Commands](https://github.com/szolotykh/kinisi-motor-controller-firmware/blob/main/commands.md)\
Follow Arduino library installation instructions to install this library manually: [Installing Libraries](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries)\
*Note: This library is not compatible with only 3.3V Arduino boards.*

API v2 is incompatible with API v1. See [protocol details](docs/protocol-v2.md).

The library bundles public profiles for all four supported Kinisi boards.
After `begin()`, `boardProfile()` exposes the automatically selected hardware
profile and `supportsSensor()` checks sensor hardware, firmware protocol and
Wire capacity. Profiles compile into C++ tables, stored in flash on AVR, with
no runtime JSON parser or download.

## Examples
Below is the example of using this library to control the status LED on the kinisi motion controller:

```cpp
#include <kinisi.h>

const int ledPin = LED_BUILTIN; // Status LED connected to pin 13
bool ledState = false; // To keep track of the LED state
KinisiController controller(8); // Initialize the Kinisi controller with the default address (8)

void setup() {
  if (!controller.begin()) {
    while (true) delay(1000); // Initialization failed.
  }
  pinMode(ledPin, OUTPUT); // Initialize the LED pin as an output
}

void loop() {
  if (!controller.poll()) return;
  static uint32_t toggled = 0;
  if (uint32_t(millis() - toggled) < 1000) return;
  toggled = millis();
  if (!controller.toggle_status_led_state()) {
    delay(1000);
    return; // Inspect controller.lastError().
  }
  
  ledState = !ledState; // Invert LED state
  digitalWrite(ledPin, ledState ? HIGH : LOW); // Update LED state
}
```
Examples can be found in [examples](examples) folder.
See the [motor/platform position example](examples/position-control/position-control.ino) for position control.
See the [sensors example](examples/sensors/sensors.ino) for temperature and IMU reads.

## Updating library
Update commands and profiles from the project root:
```bash
python tools/update-commands.py --schema ../kinisi-motor-controller-firmware/commands.json
python tools/update-board-profiles.py --profiles ../kinisi-motor-controller-firmware/include/board_profiles.json
```
The schema is explicit so regeneration cannot silently switch protocol versions.
Both scripts accept `--branch <firmware-branch>` to download their source explicitly,
or `--check` with either source option to detect stale files without writing.
The snapshots are `tools/commands.json` and `tools/board_profiles.json`; the
generated `kinisi_board_profiles.h/.cpp` ship with the library. Invalid inputs
are rejected before replacing generated files. Custom runtime profiles are not supported.

## Links
- [Kinisi Motion Controller firmware](https://github.com/szolotykh/kinisi-motor-controller-firmware)
- [Kinisi Motion Controller hardware](https://github.com/szolotykh/kinisi-motor-controller-board)
- [JavaScipt package for kinisi motor controller](https://github.com/szolotykh/jskinisi)
- [ArduinoKinisi library for kinisi motor controller](https://github.com/szolotykh/ArduinoKinisi)
- [Python package for kinisi motor controller](https://github.com/szolotykh/pykinisi)

## Wire buffer size

Temperature replies fit a default 32-byte Wire buffer. IMU replies require
**at least 52 bytes**; a **64-byte TX/RX buffer** is suitable.

`initialize_platform_position_pid_controller(...)` requires a Wire transmit buffer
of at least **100 bytes**; **128 bytes is recommended**. Configure the board's
actual Wire buffer and set `KINISI_WIRE_BUFFER_SIZE` to match. See
[Wire buffer details](docs/protocol-v2.md#wire-transaction-limits).
