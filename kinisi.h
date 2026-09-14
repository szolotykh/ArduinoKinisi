// Generated from tools/commands.json; edit the schema or generator instead.
#ifndef KINISI_H
#define KINISI_H
#include "kinisi_protocol.h"

/** Blocking I2C client. begin() completes API-v2 INIT and READY in uptime mode. */
class KinisiController : public KinisiProtocol {
public:
    /** Select a seven-bit I2C address (default 8). Does not touch the bus. */
    explicit KinisiController(uint8_t address = 8) : KinisiProtocol(address) {}

    /** This command initializes a motor and prepares it for use. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels), so platform wheels are not reconfigured out from under the platform. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, MOTOR_OWNED. Check lastError() after getters; setters return success. */
    bool initialize_motor(uint8_t motor_index, bool is_reversed);

    /** This command sets the speed of the specified motor in PWM. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels); use the platform velocity commands to drive platform wheels. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, MOTOR_OWNED, MOTOR_NOT_INITIALIZED. Check lastError() after getters; setters return success. */
    bool set_motor_speed(uint8_t motor_index, double pwm);

    /** Coasts the motor to a stop: both H-bridge outputs are driven low, leaving the motor terminals open (high impedance) so it free-wheels and spins down gradually under its own friction. This also stops that motor's closed-loop speed controller if one is running (started via INITIALIZE_MOTOR_CONTROLLER), so the PID loop cannot re-drive the motor; to command the motor by target speed again you must re-initialize its controller. This is a single-motor command and is ignored if the motor is currently owned by an active platform (one of its wheels); to stop a platform, use STOP_PLATFORM_CONTROLLER, COAST_PLATFORM or BRAKE_PLATFORM instead. Use STOP_MOTOR for a soft, low-stress stop; use BRAKE_MOTOR when you need the motor to hold position and stop quickly. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, MOTOR_OWNED. Check lastError() after getters; setters return success. */
    bool stop_motor(uint8_t motor_index);

    /** Actively brakes the motor (short brake): both H-bridge outputs are driven high, shorting the motor terminals together so the motor's own back-EMF resists rotation and it stops quickly and holds position. This also stops that motor's closed-loop speed controller if one is running (started via INITIALIZE_MOTOR_CONTROLLER), so the PID loop cannot re-drive the motor; to command the motor by target speed again you must re-initialize its controller. This is a single-motor command and is ignored if the motor is currently owned by an active platform (one of its wheels); to brake a platform, use BRAKE_PLATFORM (or STOP_PLATFORM_CONTROLLER / COAST_PLATFORM) instead. Use BRAKE_MOTOR for a fast, holding stop; use STOP_MOTOR to let the motor coast freely instead. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, MOTOR_OWNED. Check lastError() after getters; setters return success. */
    bool brake_motor(uint8_t motor_index);

    /** This command sets the controller for the specified motor. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels), so it cannot create a competing controller on a platform wheel. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, MOTOR_OWNED. Check lastError() after getters; setters return success. */
    bool initialize_motor_controller(uint8_t motor_index, bool is_reversed, uint8_t encoder_index, bool is_encoder_reversed, double encoder_resolution, double kp, double ki, double kd, double integral_limit);

    /** This command sets the target speed for the specified motor in radians. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels); use SET_PLATFORM_TARGET_VELOCITY to drive platform wheels. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, MOTOR_OWNED, CONTROLLER_NOT_INITIALIZED. Check lastError() after getters; setters return success. */
    bool set_motor_target_speed(uint8_t motor_index, double speed);

    /** This command resets the closed-loop controller for the specified motor: it clears the accumulated PID state (integrator windup, derivative history and internal output) and re-zeros the target speed, while keeping the controller running with its existing tuning (kp/ki/kd). Use it to recover from integrator windup or to bring a motor cleanly to a stop without deleting and re-initializing the controller. No effect if no controller is running for that motor, and ignored if the motor is currently owned by an active platform (one of its wheels). Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, MOTOR_OWNED. Check lastError() after getters; setters return success. */
    bool reset_motor_controller(uint8_t motor_index);

    /** This command gets the state of the controller for the specified motor. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    motor_controller_state get_motor_controller_state(uint8_t motor_index);

    /** This command deletes the controller for the specified motor. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels); use STOP_PLATFORM_CONTROLLER to stop the platform controller instead. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, MOTOR_OWNED. Check lastError() after getters; setters return success. */
    bool delete_motor_controller(uint8_t motor_index);

    /** This command sets the global update frequency (in Hz) of the closed-loop motor controller task. All motor controllers share a single control loop, so this frequency is global and affects every currently running controller as well as any created afterwards; the PID sampling time is updated to match. The requested value is clamped to the supported range of 1 to 1000 Hz (the 1000 Hz maximum is bounded by the 1 ms RTOS tick). The value is then quantized to the 1 ms RTOS tick (period_ms = 1000 / frequency), so effective frequencies are 1000/N Hz. A value of 0 is invalid and ignored. Defaults to 10 Hz (100 ms) at start-up. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool set_controller_frequency(uint16_t frequency);

    /** This command retrieves the current global update frequency (in Hz) of the closed-loop motor controller task. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    uint16_t get_controller_frequency();

    /** This command initializes an encoder and prepares it for use. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool initialize_encoder(uint8_t encoder_index, double encoder_resolution, bool is_reversed);

    /** This command retrieves the current value of the encoder. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, ENCODER_NOT_INITIALIZED. Check lastError() after getters; setters return success. */
    uint16_t get_encoder_value(uint8_t encoder_index);

    /** This command starts the odometry calculation for the specified encoder. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CLOCK_NOT_READY, ENCODER_NOT_INITIALIZED. Check lastError() after getters; setters return success. */
    bool start_encoder_odometry(uint8_t encoder_index);

    /** This command resets the odometry calculation for the specified encoder. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool reset_encoder_odometry(uint8_t encoder_index);

    /** This command stops the odometry calculation for the specified encoder. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool stop_encoder_odometry(uint8_t encoder_index);

    /** This command retrieves the odometry of the specified encoder. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CLOCK_NOT_READY, ODOMETRY_NOT_INITIALIZED, SAMPLE_NOT_AVAILABLE. Check lastError() after getters; setters return success. */
    encoder_odometry_sample get_encoder_odometry(uint8_t encoder_index);

    /** This command sets the global update frequency (in Hz) of the odometry task. A single odometry task integrates all encoder and platform odometry, so this frequency is global. The requested value is clamped to the supported range of 1 to 1000 Hz (the 1000 Hz maximum is bounded by the 1 ms RTOS tick). The value is then quantized to the 1 ms RTOS tick (period_ms = 1000 / frequency), so effective frequencies are 1000/N Hz. A value of 0 is invalid and ignored. Defaults to 20 Hz (50 ms) at start-up. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool set_odometry_frequency(uint16_t frequency);

    /** This command retrieves the current global update frequency (in Hz) of the odometry task. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    uint16_t get_odometry_frequency();

    /** This command initializes a digital pin and prepares it for use. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool initialize_gpio_pin(uint8_t pin_number, uint8_t mode);

    /** This command sets the specified pin to a state. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool set_gpio_pin_state(uint8_t pin_number, uint8_t state);

    /** This command gets the state of the specified pin. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    uint8_t get_gpio_pin_state(uint8_t pin_number);

    /** This command toggles the specified pin. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool toggle_gpio_pin_state(uint8_t pin_number);

    /** This command sets the status LED to a state. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool set_status_led_state(uint8_t state);

    /** This command toggles the status LED. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool toggle_status_led_state();

    /** This command initializes a mecanum (4-wheel) platform and prepares it for use. It uses motor and encoder indices 0, 1, 2 and 3 (one per wheel), which correspond to the is_reversed_0..3 and is_encoder_reversed_0..3 parameters. All four motor slots are occupied by this platform. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool initialize_mecanum_platform(bool is_reversed_0, bool is_reversed_1, bool is_reversed_2, bool is_reversed_3, bool is_encoder_reversed_0, bool is_encoder_reversed_1, bool is_encoder_reversed_2, bool is_encoder_reversed_3, double length, double width, double wheels_diameter, double encoder_resolution);

    /** This command initializes an omni (3-wheel) platform and prepares it for use. It uses motor and encoder indices 0, 1 and 2 (one per wheel), which correspond to the is_reversed_0..2 and is_encoder_reversed_0..2 parameters. Motor index 3 is not used by this platform and stays free for other purposes. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool initialize_omni_platform(bool is_reversed_0, bool is_reversed_1, bool is_reversed_2, bool is_encoder_reversed_0, bool is_encoder_reversed_1, bool is_encoder_reversed_2, double wheels_diameter, double robot_radius, double encoder_resolution);

    /** This command initializes a differential (2-wheel) platform and prepares it for use. It uses motor and encoder index 0 for the left wheel and index 1 for the right wheel, which correspond to the is_reversed_0/1 and is_encoder_reversed_0/1 parameters. Motor indices 2 and 3 are not used by this platform and stay free for other purposes. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool initialize_differential_platform(bool is_reversed_0, bool is_reversed_1, bool is_encoder_reversed_0, bool is_encoder_reversed_1, double wheel_diameter, double wheel_base, double encoder_resolution);

    /** This command sets the velocity for the platform in PWM. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, PLATFORM_NOT_INITIALIZED. Check lastError() after getters; setters return success. */
    bool set_platform_velocity(double x, double y, double t);

    /** This command sets the controller for the platform. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, PLATFORM_NOT_INITIALIZED. Check lastError() after getters; setters return success. */
    bool start_platform_controller(double kp, double ki, double kd, double integral_limit);

    /** This command set the target velocity for the platform in meters per second. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, PLATFORM_NOT_INITIALIZED, CONTROLLER_NOT_INITIALIZED. Check lastError() after getters; setters return success. */
    bool set_platform_target_velocity(double x, double y, double t);

    /** This command gets the current velocity of the platform in meters per second. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    platform_velocity get_platform_current_velocity();

    /** This command stops the controller for the platform. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool stop_platform_controller();

    /** This command starts the odometry calculation for the platform. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CLOCK_NOT_READY, PLATFORM_NOT_INITIALIZED. Check lastError() after getters; setters return success. */
    bool start_platform_odometry();

    /** This command resets the odometry calculation for the platform. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool reset_platform_odometry();

    /** This command stops the odometry calculation for the platform. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool stop_platform_odometry();

    /** This command retrieves the odometry of the platform in meters and radians. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CLOCK_NOT_READY, ODOMETRY_NOT_INITIALIZED, SAMPLE_NOT_AVAILABLE. Check lastError() after getters; setters return success. */
    platform_odometry_sample get_platform_odometry();

    /** This command actively brakes all of this platform's wheel motors (short brake) so they resist motion and hold position, and stops the platform velocity controller if it is running (you must call START_PLATFORM_CONTROLLER again to resume closed-loop platform control). Motors used outside this platform are not affected. The motors resist motion until a new command is issued. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool brake_platform();

    /** This command lets all of this platform's wheel motors coast freely (high impedance) so they spin down without resistance, and stops the platform velocity controller if it is running (you must call START_PLATFORM_CONTROLLER again to resume closed-loop platform control). Motors used outside this platform are not affected. The motors spin down without resistance. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool coast_platform();

    /** Set independent time-sync refresh interval for this connection. Default 30000 ms. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool set_time_sync_interval(uint32_t interval_ms);

    /** Read this connection's clock mode, quality, interval and age. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    time_status get_time_status();
};
#endif
