// File: examples/encoder/encoder.ino
// Read timestamped encoder odometry using controller uptime (no RTC required).
#include <kinisi.h>

KinisiController controller(8);
const uint8_t encoderIndex = 0;

/** Complete INIT/READY, then initialize and start encoder odometry. */
void setup() {
  Serial.begin(9600);
  if (!controller.begin() ||
      !controller.initialize_encoder(encoderIndex, 1000.0, false) ||
      !controller.start_encoder_odometry(encoderIndex)) {
    Serial.println("Initialization failed; check lastError(), wiring and firmware.");
    while (true) delay(1000);
  }
}

/** Publish only valid samples; the first sample may not be ready immediately. */
void loop() {
  encoder_odometry_sample sample = controller.get_encoder_odometry(encoderIndex);
  if (controller.lastError().failure == KinisiFailure::NONE) {
    // Arduino Print does not consistently support uint64_t, so show uptime seconds.
    Serial.print("Acquired at controller uptime (seconds): ");
    Serial.println(static_cast<unsigned long>(sample.timestamp_us / 1000000ULL));
    Serial.print("Angle (radians): ");
    Serial.println(sample.angle);
  } else {
    Serial.println("No valid odometry sample; inspect controller.lastError().");
  }
  delay(1000);
}
