# ArduinoKinisi
Arduino library for kinisi motion controller. This library allows to control kinisi motion controller from Arduino board via I2C interface.\
Discription of the motor controller commands can be find here: [Kinisi Motion Controller Commands](https://github.com/szolotykh/kinisi-motor-controller-firmware/blob/main/commands.md)\
Follow Arduino library installation instructions to install this library manually: [Installing Libraries](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries)\
*Note: This library is not compatible with only 3.3V Arduino boards.*

API **2.3.1 is incompatible with API v1**. `begin()` now completes INIT/READY and
returns success. Arduino advertises no wall-clock capability; odometry timestamps
are controller uptime in microseconds. See [protocol details](docs/protocol-v2.md)
for errors, response types, and Wire buffer requirements.

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

## Updating library
To update the library, run the following python script:
```bash
python tools/update-commands.py --schema ../kinisi-motor-controller-firmware/commands.json
```
The schema is explicit so regeneration cannot silently switch protocol versions.
Use `--branch <firmware-branch>` instead to download a selected firmware schema.

## Links
- [Kinisi Motion Controller firmware](https://github.com/szolotykh/kinisi-motor-controller-firmware)
- [Kinisi Motion Controller hardware](https://github.com/szolotykh/kinisi-motor-controller-board)
- [JavaScipt package for kinisi motor controller](https://github.com/szolotykh/jskinisi)
- [ArduinoKinisi library for kinisi motor controller](https://github.com/szolotykh/ArduinoKinisi)
- [Python package for kinisi motor controller](https://github.com/szolotykh/pykinisi)

## Velocity and position control

The library exposes the firmware 2.3.1 command set and still connects to protocol
2.1+. Motor/platform position requires 2.2+, and full position PID setup requires
2.3+. Unsupported position calls return `false` with `UNSUPPORTED_COMMAND`
without sending a frame or closing the session.

`initialize_motor_controller` and `start_platform_controller` default their final
`integral_limit` argument to **100 PWM percentage points**. It bounds I alone;
total PWM stays capped at +/-100% with firmware anti-windup. Gains and limit are
validated before transmission. Firmware 2.3.1 uses direct PID output, so retune
older gains. Kp=1, Ki=1, Kd=0 are example starting values, not a tuned motor profile.

Initialize velocity before position. Motor position uses continuous radians;
platform targets use world-frame meters/meters/radians. Position integral limits
are speed contributions (rad/s or m/s). Reset changes the origin and clears the
target. Velocity overrides cancel position mode; stops and reinitialization can
require position setup again. See [the motor/platform example](examples/position-control/position-control.ino).

**Buffer requirement for `initialize_platform_position_pid_controller(...)`:**
This command sends all 12 parameters in one **100-byte I2C frame** (96 bytes of
parameters plus a 4-byte header). The Arduino board's actual Wire transmit buffer
must hold at least 100 bytes; **128 bytes is recommended**. Set
`KINISI_WIRE_BUFFER_SIZE` consistently for the library build; changing this macro
alone does not enlarge the board's Wire buffer.

With a configured 32- or 64-byte buffer, this call returns `false` with
`controller.lastError().failure` set to `KinisiFailure::FRAME_TOO_LARGE`, and
nothing is transmitted. Message chunking is not supported, so the command cannot
be split across Wire transactions. This requirement affects sending the setup
command, not the firmware's PID update rate. See [Wire buffer details](docs/protocol-v2.md).

A 64-byte buffer supports motor position PID initialization; a 32-byte buffer
cannot.
