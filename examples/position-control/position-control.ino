// Motor demo needs real Wire TX/RX buffers of at least 64 bytes.
// Platform demo needs at least 100-byte TX and 38-byte RX (use 128-byte buffers).
// Set KINISI_WIRE_BUFFER_SIZE for the library build to match real core buffers;
// defining it in this sketch alone does not resize Wire or the library.
#include <kinisi.h>

KinisiController controller(8);
const bool platformDemo = false; // Change only after configuring platform hardware.
bool moving = false;
uint32_t started = 0;

void setup() {
  if (!controller.begin()) return;
  const init_response& info = controller.boardInfo();
  if (info.protocol_minor < 3 || (info.protocol_minor == 3 && info.protocol_patch < 1)) return;
  // Starting gains only: tune velocity and verify encoder direction first.
  if (platformDemo) {
    if (!controller.initialize_omni_platform(false,false,false,false,false,false,0.1,0.15,1425.1)) return;
    if (!controller.start_platform_controller(1,1,0)) return; // Default I limit: 100% PWM.
    if (!controller.initialize_platform_position_pid_controller(1,2,0.2,0.5,0.01,0.03,0,0,0.2,0,0,0.5)) return;
    if (!controller.reset_platform_position()) return;
    delay(100); // Allow fresh odometry after reset; remain within heartbeat timeout.
    if (!controller.poll()) return;
    controller.get_platform_odometry();
    if (controller.lastError().failure != KinisiFailure::NONE) return;
    if (!controller.set_platform_position(0.1,0,0.2)) return; // meters, meters, radians.
  } else {
    if (!controller.initialize_motor_controller(0,false,0,false,1425.1,1,1,0)) return;
    if (!controller.initialize_motor_position_pid_controller(0,2,1,0.02,0,0,1)) return;
    if (!controller.reset_motor_position(0)) return;
    if (!controller.set_motor_position(0,1.0)) return; // Continuous radians.
  }
  moving = true;
  started = millis();
}

void loop() {
  if (!controller.poll()) { moving = false; return; }
  if (moving && uint32_t(millis()-started) >= 5000) {
    if (platformDemo) controller.stop_platform_controller();
    else controller.delete_motor_controller(0);
    moving = false;
  }
}
