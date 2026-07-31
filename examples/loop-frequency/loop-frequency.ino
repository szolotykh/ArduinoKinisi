#include <kinisi.h>

KinisiController controller(8); // Initialize the Kinisi controller with the default address (8)

void setup() {
  Serial.begin(9600);  // Initialize serial communication for debugging
  controller.begin(); // Start the I2C communication

  // Read the current loop frequencies.
  Serial.println("Controller frequency: " + String(controller.get_controller_frequency()) + " Hz");
  Serial.println("Odometry frequency: " + String(controller.get_odometry_frequency()) + " Hz");

  // Update the loop frequencies (Hz, 1-1000; values outside are clamped, 0 is ignored).
  controller.set_controller_frequency(100);
  controller.set_odometry_frequency(50);

  // Read back to confirm.
  Serial.println("New controller frequency: " + String(controller.get_controller_frequency()) + " Hz");
  Serial.println("New odometry frequency: " + String(controller.get_odometry_frequency()) + " Hz");
}

void loop() {
  // Nothing to do here.
  delay(1000);
}
