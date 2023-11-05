# ArduinoKinisi
Arduino library for kinisi motion controller. This library allows to control kinisi motion controller from Arduino board via I2C interface.\
Discription of the motor controller commands can be find here: [Kinisi Motion Controller Commands](https://github.com/szolotykh/kinisi-motor-controller-firmware/blob/main/commands.md)\
Follow Arduino library installation instructions to install this library manually: [Installing Libraries](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries)\
*Note: This library is not compatible with only 3.3V Arduino boards.*

## Examples
Below is the example of using this library to control the status LED on the kinisi motion controller:

```cpp
#include <kinisi.h>

const int ledPin = LED_BUILTIN; // Status LED connected to pin 13
bool ledState = false; // To keep track of the LED state
KinisiController controller(8); // Initialize the Kinisi controller with the default address (8)

void setup() {
  controller.begin(); // Start the I2C communication
  pinMode(ledPin, OUTPUT); // Initialize the LED pin as an output
}

void loop() {
  controller.toggle_status_led_state(); // Send toggle command to the Kinisi controller
  
  ledState = !ledState; // Invert LED state
  digitalWrite(ledPin, ledState ? HIGH : LOW); // Update LED state
  delay(1000); // Wait for 1 second
} 
```
Examples can be found in [examples](examples) folder.

## Updating library
To update the library, run the following python script:
```bash
cd ./tools
python update_commands.py
```
This script automatically downloads the latest version of the kinisi motion controller commands from firmware repository and generates the library code based on ```commands.json``` file.

## Links
- [Kinisi Motion Controller firmware](https://github.com/szolotykh/kinisi-motor-controller-firmware)
- [Kinisi Motion Controller hardware](https://github.com/szolotykh/kinisi-motor-controller-board)
- [JavaScipt package for kinisi motor controller](https://github.com/szolotykh/jskinisi)
- [ArduinoKinisi library for kinisi motor controller](https://github.com/szolotykh/ArduinoKinisi)
- [Python package for kinisi motor controller](https://github.com/szolotykh/pykinisi)