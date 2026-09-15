// File: examples/blink-command/blink-command.ino
// Toggle the board LED after the API-v2 handshake and successful ACKs.
#include <kinisi.h>

const int ledPin = LED_BUILTIN; // Status LED connected to pin 13
bool ledState = false; // To keep track of the LED state
KinisiController controller(8); // Initialize the Kinisi controller with the default address (8)

/** Establish an uptime session before sending LED commands. */
void setup() {
  if (!controller.begin()) { // API v2 INIT + READY, controller uptime
    while (true) delay(1000); // Check wiring and firmware version before retrying.
  }
  pinMode(ledPin, OUTPUT); // Initialize the LED pin as an output
}

/** Mirror the controller LED locally only when its command succeeds. */
void loop() {
  if (!controller.poll()) return;
  static uint32_t toggled = 0;
  if (uint32_t(millis() - toggled) < 1000) return;
  toggled = millis();
  if (!controller.toggle_status_led_state()) {
    delay(1000);
    return; // Inspect lastError() before deciding whether to reconnect.
  }
  
  ledState = !ledState; // Invert LED state
  digitalWrite(ledPin, ledState ? HIGH : LOW); // Update LED state
}
