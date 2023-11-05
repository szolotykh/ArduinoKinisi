#include <kinisi.h>

KinisiController controller(8); // Initialize the Kinisi controller with the default address (8)
uint8_t encoderIndex = 0; // Encoder index

void setup() {
  Serial.begin(9600);  // Initialize serial communication for debugging
  controller.begin(); // Start the I2C communication

  // Initialize the encoder
  controller.initialize_encoder(encoderIndex);
}

void loop() {
  // Get the encoder value from the Kinisi controller
  uint32_t value = controller.get_encoder_value(encoderIndex);
  Serial.println("Encoder value: " + String(value));

  // Send toggle command to the Kinisi controller
  controller.toggle_status_led_state();

  // Wait for 1 second
  delay(1000); 
} 