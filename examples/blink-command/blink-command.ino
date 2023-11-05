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