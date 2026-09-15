// File: examples/loop-frequency/loop-frequency.ino
// Configure loop frequencies and check protocol-v2 ACKs and getter errors.
#include <kinisi.h>

KinisiController controller(8);

/** Initialize the session, set both rates, and report confirmed values. */
void setup() {
  Serial.begin(9600);
  if (!controller.begin() || !controller.set_controller_frequency(100) ||
      !controller.set_odometry_frequency(50)) {
    Serial.println("Setup failed; inspect controller.lastError().");
    return;
  }
  uint16_t controllerFrequency = controller.get_controller_frequency();
  if (controller.lastError().failure != KinisiFailure::NONE) return;
  uint16_t odometryFrequency = controller.get_odometry_frequency();
  if (controller.lastError().failure != KinisiFailure::NONE) return;
  Serial.println("Controller frequency: " + String(controllerFrequency) + " Hz");
  Serial.println("Odometry frequency: " + String(odometryFrequency) + " Hz");
}

/** Continue servicing the connection watchdog independently of uptime timestamps. */
void loop() { controller.poll(); }
