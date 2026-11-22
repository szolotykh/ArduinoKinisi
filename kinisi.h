// Generated from tools/commands.json; edit the schema or generator instead.
#ifndef KINISI_H
#define KINISI_H
#include "kinisi_protocol.h"

/** Blocking I2C client. begin() completes API-v2 INIT and READY in uptime mode. */
class KinisiController : public KinisiProtocol {
public:
    /** Select a seven-bit I2C address (default 8). Does not touch the bus. */
    explicit KinisiController(uint8_t address = 8) : KinisiProtocol(address) {}
    bool subscribe(KinisiTopic topic, uint8_t index, uint32_t poll_interval_ms, KinisiSubscriptionTrigger trigger, KinisiSubscriptionDelivery delivery, uint32_t max_silence_ms = 0) { return subscribe_configured(topic, index, poll_interval_ms, trigger, delivery, max_silence_ms); }

    /** Read the latest board temperature in degrees Celsius. Sensor 0 is the TMP102 on v0.4. Sampling runs at 4 Hz; faster reads may repeat a sample. Returns SAMPLE_NOT_AVAILABLE during the first 300 ms after sensor initialization or recovery, SENSOR_NOT_AVAILABLE when the board has no such sensor, and INTERNAL_ERROR on I2C or configuration failure. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, SENSOR_NOT_AVAILABLE, SAMPLE_NOT_AVAILABLE, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    double get_temperature(uint8_t sensor_index);

    /** Read the latest accelerometer and gyroscope samples in native sensor axes. Acceleration is in m/s^2 and includes gravity; angular velocity is in rad/s. BMI088 uses +/-6 g and +/-500 degrees/s at 100 Hz. Each three-axis sample is coherent, but accel and gyro are not time-synchronized. Faster requests may repeat samples. No orientation or calibration is applied. Returns SAMPLE_NOT_AVAILABLE during the 50 ms settling interval after initialization or recovery, SENSOR_NOT_AVAILABLE for an absent sensor, and INTERNAL_ERROR on SPI, identity or configuration failure. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, SENSOR_NOT_AVAILABLE, SAMPLE_NOT_AVAILABLE, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    imu_sample get_imu(uint8_t sensor_index);

    /** Read the filtered motor-current estimate in amperes. The unsigned reading measures driver high-side current; PWM decay intervals and the analog filter affect it. Reading does not enable or initialize the motor. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, SENSOR_NOT_AVAILABLE, SAMPLE_OUT_OF_RANGE, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    double get_motor_current(uint8_t motor_index);

    /** Read motor-driver fault status without waking, initializing, enabling or clearing the driver. Detailed flags are meaningful only when status_valid is true. An unavailable SPI response cannot distinguish missing motor power from a communication or driver failure. Flags may retain latched faults until explicit motor reinitialization. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, SENSOR_NOT_AVAILABLE. Check lastError() after getters; setters return success. */
    motor_fault_status get_motor_faults(uint8_t motor_index);

    /** This command initializes a motor and prepares it for use. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels), so platform wheels are not reconfigured out from under the platform. Stops the motor velocity and position controllers before applying the command. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, MOTOR_OWNED, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool initialize_motor(uint8_t motor_index, bool is_reversed);

    /** This command sets the speed of the specified motor in PWM. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels); use the platform velocity commands to drive platform wheels. Stops the motor velocity and position controllers before applying the command. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, MOTOR_OWNED, MOTOR_NOT_INITIALIZED, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool set_motor_speed(uint8_t motor_index, double pwm);

    /** Stops the motor's velocity and position controllers and lets the motor coast with the H-bridge outputs in high impedance. On dual-PWM boards both PWM control inputs are low. On V4 enable/direction boards DISABLE is high and EN duty is zero. The command ID and payload are the same for all boards; firmware selects the hardware behavior from the board configuration. A motor owned by an active platform is rejected with MOTOR_OWNED; use COAST_PLATFORM or STOP_PLATFORM_CONTROLLER for platform wheels. Re-initialize the motor controller before using target-speed control again. Use BRAKE_MOTOR for a short brake instead of coasting. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, MOTOR_OWNED. Check lastError() after getters; setters return success. */
    bool stop_motor(uint8_t motor_index);

    /** Stops the motor's velocity and position controllers and applies a short brake, connecting both motor terminals to the same bridge rail to resist rotation. On dual-PWM boards both PWM control inputs are high. On V4 enable/direction boards DISABLE is low and EN duty is zero; direction does not affect braking. The command ID and payload are the same for all boards; firmware selects the hardware behavior from the board configuration. Short braking does not regulate or hold a specific position. A motor owned by an active platform is rejected with MOTOR_OWNED; use BRAKE_PLATFORM for platform wheels. Re-initialize the motor controller before using target-speed control again. Use STOP_MOTOR to coast instead. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, MOTOR_OWNED. Check lastError() after getters; setters return success. */
    bool brake_motor(uint8_t motor_index);

    /** This command sets the controller for the specified motor. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels), so it cannot create a competing controller on a platform wheel. Firmware 2.3.1 uses direct P+I+D PWM output with saturation anti-windup; retune gains from earlier builds. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, MOTOR_OWNED, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool initialize_motor_controller(uint8_t motor_index, bool is_reversed, uint8_t encoder_index, bool is_encoder_reversed, double encoder_resolution, double kp, double ki, double kd, double integral_limit = 100.0);

    /** This command sets the target speed for the specified motor in radians. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels); use SET_PLATFORM_TARGET_VELOCITY to drive platform wheels. Suspends position mode; a new SET_MOTOR_POSITION reactivates it. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, MOTOR_OWNED, CONTROLLER_NOT_INITIALIZED, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool set_motor_target_speed(uint8_t motor_index, double speed);

    /** This command resets the closed-loop controller for the specified motor: it clears the accumulated PID state (integrator windup, derivative history and internal output) and re-zeros the target speed, while keeping the controller running with its existing tuning (kp/ki/kd). Use it to recover from integrator windup or to bring a motor cleanly to a stop without deleting and re-initializing the controller. No effect if no controller is running for that motor, and ignored if the motor is currently owned by an active platform (one of its wheels). Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, MOTOR_OWNED, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool reset_motor_controller(uint8_t motor_index);

    /** This command gets the state of the controller for the specified motor. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    motor_controller_state get_motor_controller_state(uint8_t motor_index);

    /** This command deletes the controller for the specified motor. Rejected with MOTOR_OWNED if the motor is currently owned by an active platform (one of its wheels); use STOP_PLATFORM_CONTROLLER to stop the platform controller instead. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, MOTOR_OWNED, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool delete_motor_controller(uint8_t motor_index);

    /** This command sets the global update frequency (in Hz) of the closed-loop motor controller task. All motor controllers share a single control loop, so this frequency is global and affects every currently running controller as well as any created afterwards; the PID sampling time is updated to match. The requested value is clamped to the supported range of 1 to 1000 Hz (the 1000 Hz maximum is bounded by the 1 ms RTOS tick). The value is then quantized to the 1 ms RTOS tick (period_ms = 1000 / frequency), so effective frequencies are 1000/N Hz. A value of 0 is invalid and ignored. Defaults to 10 Hz (100 ms) at start-up. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool set_controller_frequency(uint16_t frequency);

    /** This command retrieves the current global update frequency (in Hz) of the closed-loop motor controller task. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    uint16_t get_controller_frequency();

    /** This command initializes an encoder and prepares it for use. Stops controllers using this encoder before changing its configuration. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool initialize_encoder(uint8_t encoder_index, double encoder_resolution, bool is_reversed);

    /** This command retrieves the current value of the encoder. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, ENCODER_NOT_INITIALIZED, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    uint16_t get_encoder_value(uint8_t encoder_index);

    /** This command starts the odometry calculation for the specified encoder. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CLOCK_NOT_READY, ENCODER_NOT_INITIALIZED. Check lastError() after getters; setters return success. */
    bool start_encoder_odometry(uint8_t encoder_index);

    /** This command resets the odometry calculation for the specified encoder. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool reset_encoder_odometry(uint8_t encoder_index);

    /** This command stops the odometry calculation for the specified encoder. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool stop_encoder_odometry(uint8_t encoder_index);

    /** This command retrieves the odometry of the specified encoder. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CLOCK_NOT_READY, ODOMETRY_NOT_INITIALIZED, SAMPLE_NOT_AVAILABLE. Check lastError() after getters; setters return success. */
    encoder_odometry_sample get_encoder_odometry(uint8_t encoder_index);

    /** This command sets the global update frequency (in Hz) of the odometry task. A single odometry task integrates all encoder and platform odometry, so this frequency is global. The requested value is clamped to the supported range of 1 to 1000 Hz (the 1000 Hz maximum is bounded by the 1 ms RTOS tick). The value is then quantized to the 1 ms RTOS tick (period_ms = 1000 / frequency), so effective frequencies are 1000/N Hz. A value of 0 is invalid and ignored. Defaults to 20 Hz (50 ms) at start-up. Rejected if the resulting calculation period exceeds half of any active subscription interval on either transport. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool set_odometry_frequency(uint16_t frequency);

    /** This command retrieves the current global update frequency (in Hz) of the odometry task. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    uint16_t get_odometry_frequency();

    /** This command initializes a digital pin and prepares it for use. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool initialize_gpio_pin(uint8_t pin_number, uint8_t mode);

    /** This command sets the specified pin to a state. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool set_gpio_pin_state(uint8_t pin_number, uint8_t state);

    /** This command gets the state of the specified pin. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    uint8_t get_gpio_pin_state(uint8_t pin_number);

    /** This command toggles the specified pin. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool toggle_gpio_pin_state(uint8_t pin_number);

    /** This command sets the status LED to a state. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool set_status_led_state(uint8_t state);

    /** This command toggles the status LED. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool toggle_status_led_state();

    /** This command initializes a mecanum (4-wheel) platform and prepares it for use. It uses motor and encoder indices 0, 1, 2 and 3 (one per wheel), which correspond to the is_reversed_0..3 and is_encoder_reversed_0..3 parameters. All four motor slots are occupied by this platform. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool initialize_mecanum_platform(bool is_reversed_0, bool is_reversed_1, bool is_reversed_2, bool is_reversed_3, bool is_encoder_reversed_0, bool is_encoder_reversed_1, bool is_encoder_reversed_2, bool is_encoder_reversed_3, double length, double width, double wheels_diameter, double encoder_resolution);

    /** This command initializes an omni (3-wheel) platform and prepares it for use. It uses motor and encoder indices 0, 1 and 2 (one per wheel), which correspond to the is_reversed_0..2 and is_encoder_reversed_0..2 parameters. Motor index 3 is not used by this platform and stays free for other purposes. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool initialize_omni_platform(bool is_reversed_0, bool is_reversed_1, bool is_reversed_2, bool is_encoder_reversed_0, bool is_encoder_reversed_1, bool is_encoder_reversed_2, double wheels_diameter, double robot_radius, double encoder_resolution);

    /** This command initializes a differential (2-wheel) platform and prepares it for use. It uses motor and encoder index 0 for the left wheel and index 1 for the right wheel, which correspond to the is_reversed_0/1 and is_encoder_reversed_0/1 parameters. Motor indices 2 and 3 are not used by this platform and stay free for other purposes. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool initialize_differential_platform(bool is_reversed_0, bool is_reversed_1, bool is_encoder_reversed_0, bool is_encoder_reversed_1, double wheel_diameter, double wheel_base, double encoder_resolution);

    /** This command sets the velocity for the platform in PWM. Stops platform velocity and position control before applying PWM. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, PLATFORM_NOT_INITIALIZED, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool set_platform_velocity(double x, double y, double t);

    /** This command sets the controller for the platform. Firmware 2.3.1 uses direct P+I+D PWM output with saturation anti-windup; retune gains from earlier builds. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, ENCODER_NOT_INITIALIZED, PLATFORM_NOT_INITIALIZED, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool start_platform_controller(double kp, double ki, double kd, double integral_limit = 100.0);

    /** This command set the target velocity for the platform in meters per second. Cancels platform position control; initialize it again before another pose target. With an active fusion experiment, rejects invalid or stale feedback and targets fenced by an earlier fault with SAMPLE_NOT_AVAILABLE; both experimental modes require 100 ms of observed unchanged configured counters for initial rest arming. Once armed, ordinary target updates remain accepted while moving. Wait for healthy feedback and submit a fresh target; stopped cold configuration can ACK while still WAITING. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, PLATFORM_NOT_INITIALIZED, CONTROLLER_NOT_INITIALIZED, INIT_REQUIRED, SAMPLE_NOT_AVAILABLE. Check lastError() after getters; setters return success. */
    bool set_platform_target_velocity(double x, double y, double t);

    /** This command gets the current velocity of the platform in meters per second. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    platform_velocity get_platform_current_velocity();

    /** This command stops the controller for the platform. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool stop_platform_controller();

    /** This command starts the odometry calculation for the platform. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CLOCK_NOT_READY, ENCODER_NOT_INITIALIZED, PLATFORM_NOT_INITIALIZED. Check lastError() after getters; setters return success. */
    bool start_platform_odometry();

    /** This command resets the odometry calculation for the platform. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool reset_platform_odometry();

    /** This command stops the odometry calculation for the platform. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool stop_platform_odometry();

    /** This command retrieves the odometry of the platform in meters and radians. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CLOCK_NOT_READY, ODOMETRY_NOT_INITIALIZED, SAMPLE_NOT_AVAILABLE. Check lastError() after getters; setters return success. */
    platform_odometry_sample get_platform_odometry();

    /** Stops the platform velocity and position controllers and applies a short brake to this platform's wheel motors using the selected board's motor control model. Other motors are not affected. The brake resists motion until a new command is issued; it does not regulate or hold a specific position. Call START_PLATFORM_CONTROLLER again to resume closed-loop platform control. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool brake_platform();

    /** This command lets all of this platform's wheel motors coast freely (high impedance) so they spin down without resistance, and stops the platform velocity controller if it is running (you must call START_PLATFORM_CONTROLLER again to resume closed-loop platform control). Motors used outside this platform are not affected. The motors spin down without resistance. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool coast_platform();

    /** Set independent time-sync refresh interval for this connection. Default 30000 ms. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool set_time_sync_interval(uint32_t interval_ms);

    /** Read this connection's clock mode, quality, interval and age. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    time_status get_time_status();

    /** Refresh connection activity and receive an empty ACK. Any structurally valid client command also refreshes activity; malformed frames do not. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    bool ping();

    /** Configure this session watchdog after READY. Timeout coasts all motors, clears subscriptions and requires a new INIT before further operations. Disabling also removes this session subscriptions. INIT resets to disabled, 500 ms. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CLOCK_NOT_READY. Check lastError() after getters; setters return success. */
    bool set_heartbeat_config(bool enabled, uint32_t timeout_ms);

    /** Read this session watchdog configuration. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED. Check lastError() after getters; setters return success. */
    heartbeat_config get_heartbeat_config();

    /** Create or replace this connection's subscription for an odometry source. Requires READY, enabled heartbeat and running odometry. The interval must be at least twice the calculation period. Stop or reset pauses delivery until a fresh sample is available. Errors: BOARD_IDLE, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CLOCK_NOT_READY, ODOMETRY_NOT_INITIALIZED.
     * @deprecated Retired since protocol 2.6.0. Use SUBSCRIBE. Kept for compatibility; eligible for removal in protocol 3.0.0. Check lastError() after getters; setters return success. */
    bool subscribe_odometry(uint8_t source, uint32_t interval_ms);

    /** Remove this connection's subscription for an odometry source. Succeeds if already absent and leaves odometry calculation running. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED.
     * @deprecated Retired since protocol 2.6.0. Use UNSUBSCRIBE. Kept for compatibility; eligible for removal in protocol 3.0.0. Check lastError() after getters; setters return success. */
    bool unsubscribe_odometry(uint8_t source);

    /** Allow an I2C master to read at most one due telemetry event followed by an empty ACK. Return only the ACK when no sample is available. USB clients receive events automatically. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CLOCK_NOT_READY. Check lastError() after getters; setters return success. */
    bool poll_telemetry();

    /** Initialize a bounded proportional position loop over an already running motor velocity controller. Zero is the current encoder position; initially holds zero. Stop/delete/reinitialize of velocity control discards position tuning. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CONTROLLER_NOT_INITIALIZED, MOTOR_OWNED. Check lastError() after getters; setters return success. */
    bool initialize_motor_position_controller(uint8_t motor_index, double kp, double max_speed, double tolerance);

    /** Zero the motor position and target at the current encoder count, clear velocity PID history and output, and hold zero. Requires initialized position control. Does not reset independent encoder odometry. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CONTROLLER_NOT_INITIALIZED, MOTOR_OWNED. Check lastError() after getters; setters return success. */
    bool reset_motor_position(uint8_t motor_index);

    /** Set an absolute multi-turn angle in radians relative to the last position initialization/reset. Requires initialized position and velocity controllers. Reactivates position mode after a velocity override. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CONTROLLER_NOT_INITIALIZED, MOTOR_OWNED. Check lastError() after getters; setters return success. */
    bool set_motor_position(uint8_t motor_index, double position);

    /** Read the latest motor position in radians in the position-controller frame. Requires initialized position control. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CONTROLLER_NOT_INITIALIZED, MOTOR_OWNED. Check lastError() after getters; setters return success. */
    double get_motor_position(uint8_t motor_index);

    /** Initialize bounded pose control after START_PLATFORM_CONTROLLER. Starts odometry if needed, preserves its world frame and zeros velocity targets. No motion until SET_PLATFORM_POSITION. Supports omni, mecanum and differential bases. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CONTROLLER_NOT_INITIALIZED, PLATFORM_NOT_INITIALIZED, ENCODER_NOT_INITIALIZED. Check lastError() after getters; setters return success. */
    bool initialize_platform_position_controller(double linear_kp, double angular_kp, double max_linear_speed, double max_angular_speed, double position_tolerance, double heading_tolerance);

    /** Cancel the pose target, zero velocity targets, and reset platform odometry to (0,0,0). Keeps position tuning. Wait for a fresh odometry sample before setting another target. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CONTROLLER_NOT_INITIALIZED, PLATFORM_NOT_INITIALIZED. Check lastError() after getters; setters return success. */
    bool reset_platform_position();

    /** Set absolute (x,y,t) in the current odometry world frame: meters, meters, radians. Heading uses the shortest angular path. Requires initialized position control and fresh odometry. Differential bases approach the point before aligning final heading. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CONTROLLER_NOT_INITIALIZED, PLATFORM_NOT_INITIALIZED, ODOMETRY_NOT_INITIALIZED, SAMPLE_NOT_AVAILABLE. Check lastError() after getters; setters return success. */
    bool set_platform_position(double x, double y, double t);

    /** Initialize a position PID over the existing velocity controller. Initialize a bounded position PID loop over an already running motor velocity controller. Zero is the current encoder position; initially holds zero. Stop/delete/reinitialize of velocity control discards position tuning. Clears position PID history. Ki and Kd may be zero to disable I and D. Integral limits bound the integral velocity contribution. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CONTROLLER_NOT_INITIALIZED, MOTOR_OWNED. Check lastError() after getters; setters return success. */
    bool initialize_motor_position_pid_controller(uint8_t motor_index, double kp, double max_speed, double tolerance, double ki, double kd, double integral_limit);

    /** Initialize a position PID over the existing velocity controller. Initialize bounded pose control after START_PLATFORM_CONTROLLER. Starts odometry if needed, preserves its world frame and zeros velocity targets. No motion until SET_PLATFORM_POSITION. Supports omni, mecanum and differential bases. Clears position PID history. Ki and Kd may be zero to disable I and D. Integral limits bound the integral velocity contribution. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INTERNAL_ERROR, INIT_REQUIRED, CONTROLLER_NOT_INITIALIZED, PLATFORM_NOT_INITIALIZED, ENCODER_NOT_INITIALIZED. Check lastError() after getters; setters return success. */
    bool initialize_platform_position_pid_controller(double linear_kp, double angular_kp, double max_linear_speed, double max_angular_speed, double position_tolerance, double heading_tolerance, double linear_ki, double linear_kd, double linear_integral_limit, double angular_ki, double angular_kd, double angular_integral_limit);

    /** Create or replace this connection's subscription for a topic and resource. Requires READY and enabled heartbeat. Odometry must be running. The interval must meet the topic minimum and, for odometry, be at least twice the calculation period. Failed reads pause delivery until recovery. Errors: BOARD_IDLE, INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, ODOMETRY_NOT_INITIALIZED, SENSOR_NOT_AVAILABLE, INTERNAL_ERROR, SUBSCRIPTION_LIMIT, SAMPLE_OUT_OF_RANGE. Check lastError() after getters; setters return success. */
    bool subscribe(KinisiTopic topic_id, uint8_t index, uint32_t interval_ms, KinisiSubscriptionMode mode = KinisiSubscriptionMode::PERIODIC);

    /** Remove this connection's subscription for a topic and resource. Requires READY. Succeeds if already absent and leaves calculation running. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY. Check lastError() after getters; setters return success. */
    bool unsubscribe(KinisiTopic topic_id, uint8_t index);

    /** Create or replace this connection's subscription with a delivery mode. Requires READY and enabled heartbeat. The interval sets the sampling cadence and must meet the topic minimum; odometry also requires twice the calculation period. ON_CHANGE sends an initial snapshot and changed values, excluding timestamp and clock metadata. Failed reads pause delivery; recovery sends a fresh snapshot. Errors: BOARD_IDLE, INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, ODOMETRY_NOT_INITIALIZED, SENSOR_NOT_AVAILABLE, INTERNAL_ERROR, SUBSCRIPTION_LIMIT, SAMPLE_OUT_OF_RANGE. Check lastError() after getters; setters return success. */
    bool subscribe_with_mode(KinisiTopic topic_id, uint8_t index, uint32_t interval_ms, KinisiSubscriptionMode mode);

    /** Stop the initialized motor and its controller, clear latched driver faults, and return fresh diagnostics. Requires a ready connection. The motor remains stopped. Active conditions can remain reported. Platform-owned motors are rejected. Driver configuration is preserved; sleeping drivers are not woken. A failed clear or unreadable verification returns an error. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, SENSOR_NOT_AVAILABLE, MOTOR_OWNED, MOTOR_NOT_INITIALIZED, INTERNAL_ERROR, CLOCK_NOT_READY. Check lastError() after getters; setters return success. */
    motor_fault_status clear_motor_faults(uint8_t motor_index);

    /** Read the applied hardware current-regulation setting in milliamps. Requires a ready connection and an initialized, awake motor driver. Does not change motor state. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, MOTOR_NOT_INITIALIZED, SENSOR_NOT_AVAILABLE, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    motor_current_limit get_motor_current_limit(uint8_t motor_index);

    /** Stop the motor and its controller, apply and verify hardware current regulation, and return the applied setting. Requires a ready connection and an initialized, awake driver. Only board-supported thresholds are accepted. Platform-owned motors are rejected. The motor stays stopped. Settings are volatile; motor reinitialization applies board defaults. Driver power loss requires reinitialization. Overcurrent protection is unchanged. Errors: DRIVERS_ASLEEP, INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, MOTOR_NOT_INITIALIZED, SENSOR_NOT_AVAILABLE, INTERNAL_ERROR, MOTOR_OWNED. Check lastError() after getters; setters return success. */
    motor_current_limit set_motor_current_limit(uint8_t motor_index, bool enabled, uint16_t threshold_ma);

    /** Create or replace a subscription with independent trigger and delivery settings. Send an initial snapshot. PERIODIC uses poll_interval_ms; ON_EVENT requires zero polling interval and an event-capable source. ON_CHANGE suppresses unchanged values, excluding timestamp and clock metadata; max_silence_ms requests fresh refreshes, or zero disables refreshes. Delivery is best effort and requires READY and heartbeat. Errors: BOARD_IDLE, INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, ODOMETRY_NOT_INITIALIZED, SENSOR_NOT_AVAILABLE, INTERNAL_ERROR, SUBSCRIPTION_LIMIT, SAMPLE_OUT_OF_RANGE. Check lastError() after getters; setters return success. */
    bool subscribe_configured(KinisiTopic topic_id, uint8_t index, uint32_t poll_interval_ms, KinisiSubscriptionTrigger trigger, KinisiSubscriptionDelivery delivery, uint32_t max_silence_ms);

    /** Read the board-wide motor PWM frequency in hertz. Does not initialize motors or change outputs. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    uint32_t get_pwm_frequency();

    /** Stop all motors and motor/platform controllers, then set the board-wide PWM frequency. All motors remain stopped. The setting survives motor initialization and connection changes. At boot the board uses its default unless it restores an explicitly saved v0.4 configuration. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    uint32_t set_pwm_frequency(KinisiPwmFrequency frequency_hz);

    /** Save the complete running platform setup and motor PWM frequency to onboard FRAM. Available on v0.4 with protocol 2.14 or later. ACK confirms a durable save. A subsequent boot restores the setup with outputs stopped. Errors: INVALID_LENGTH, INIT_REQUIRED, CLOCK_NOT_READY, SENSOR_NOT_AVAILABLE, PLATFORM_NOT_INITIALIZED, INVALID_ARGUMENT, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool save_configuration();

    /** Read one field of the running v0.4 configuration. Use GET_PWM_FREQUENCY to read the board-wide switching frequency. Errors: INVALID_LENGTH, INIT_REQUIRED, CLOCK_NOT_READY, SENSOR_NOT_AVAILABLE, INVALID_ARGUMENT, PLATFORM_NOT_INITIALIZED. Check lastError() after getters; setters return success. */
    double get_configuration_field(uint8_t field, uint8_t index);

    /** Set the board-wide motor inhibit. Entry stops all controllers and blocks new motion. On v0.3 and Nano (protocol 2.17), entry holds each DRV8870 input pair low. Release keeps the inputs low until a fresh drive or explicit short brake. On v0.4 (protocol 2.15+), wake verifies retained settings for initialized drivers and leaves outputs stopped. Failed transitions retain the inhibit for explicit retry. Requires READY and a profile supporting driver sleep. Errors: DRIVERS_ASLEEP, BOARD_IDLE, INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, SENSOR_NOT_AVAILABLE, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    bool set_driver_sleep(uint8_t sleeping);

    /** Read the software motor inhibit across connections. It clears at boot. A zero permits commands and does not establish that each chip is awake. Requires READY and driver sleep support (v0.4 since protocol 2.15, original v0.3 and Nano since 2.17). Errors: INVALID_LENGTH, INIT_REQUIRED, CLOCK_NOT_READY, SENSOR_NOT_AVAILABLE. Check lastError() after getters; setters return success. */
    uint8_t get_driver_sleep();

    /** Set communication-preserving board idle. Entry (1) stops control, inhibits the drivers and permits shallow CPU sleep between tasks. USB traffic, time sync, queries and heartbeat keep using the existing session. Only explicit exit (0) clears idle. Exit preserves a prior independent driver inhibit and leaves outputs stopped. On original v0.3 and Nano both DRV8870 inputs stay low until fresh drive or brake. On v0.4, previously active drivers are verified before release. A failed transition retains idle and inhibit for explicit retry. Requires READY, an idle-capable profile and protocol 2.18 on v0.3 or Nano, or 2.16 on v0.4. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, SENSOR_NOT_AVAILABLE, INTERNAL_ERROR, BOARD_IDLE. Check lastError() after getters; setters return success. */
    bool set_board_idle(uint8_t idle);

    /** Read the shared logical idle request without clearing it. Interrupts can resume the CPU while this request remains set. GET_DRIVER_SLEEP reports the effective driver inhibit. Idle survives reconnect and clears at reset. Requires READY and board idle support (original v0.3 and Nano since protocol 2.18, v0.4 since 2.16). Errors: INVALID_LENGTH, INIT_REQUIRED, CLOCK_NOT_READY, SENSOR_NOT_AVAILABLE. Check lastError() after getters; setters return success. */
    uint8_t get_board_idle();

    /** Configure the volatile v0.4 experiment only while motion is stopped and all commanded targets and motor outputs are zero; initialized controllers with idle zero targets are allowed. Mode 0 retains the legacy encoder path (boot default), 1 uses encoder feedback with the common chassis correction, and 2 uses fused feedback with that same correction. No IMU-only mode. For modes 1/2 the caller declares a rigid, level board with component side up, IMU at the chassis reference point, and mounting_yaw_rad as the angle of board +X expressed in chassis XY, counterclockwise about chassis +Z. Sensor-to-board axes are (-x,-y,+z); chassis axes are forward/left/up. Reject nonfinite or out-of-range yaw, any observed configured encoder movement in the preceding 100 ms, or a counter change since a fresh sample. Stopped cold setup may ACK in WAITING; ACK does not establish readiness. Both experimental modes require 100 ms of observed unchanged configured counters for initial rest arming before accepting a motion target; once armed, ordinary target updates remain accepted while moving; fused bias calibration may run within that window. Reconfiguration cancels old targets. Sources may requalify automatically after an ordinary sensor fault, but healthy inputs and a fresh target are required before control resumes. Does not save to FRAM or start motion. Requires READY, exact v0.4.0 fusion capability and protocol 2.19. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, SENSOR_NOT_AVAILABLE, INTERNAL_ERROR, BOARD_IDLE, DRIVERS_ASLEEP, PLATFORM_NOT_INITIALIZED. Check lastError() after getters; setters return success. */
    bool set_fusion_experiment(uint8_t mode, double mounting_yaw_rad);

    /** Read experiment mode, health, fault, flags and timestamped sample ages without setting targets or changing configuration. A read may enforce an expired feedback deadline, clear the live target and coast. Health/fault/flag values use the FusionHealth/FusionFault/FusionFlag catalogs. Timestamp uses the monotonic board clock; unavailable ages are UINT32_MAX. The physical IMU producer waits 2 ms between passes, without guaranteeing a 2 ms acquisition cadence, and pairs only acquisition windows entirely preceding the encoder timestamp: each window is at most 5 ms wide, worst-case accel/gyro skew at most 15 ms, and age at most 35 ms. These are software acceptance limits, not a physical timing qualification. ESTIMATE_VALID describes the selected experiment estimate, not physical accuracy. Requires READY, exact v0.4.0 fusion capability and protocol 2.19. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, SENSOR_NOT_AVAILABLE, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    fusion_status get_fusion_status();

    /** Read one compact experiment vector: 0 encoder pose (relative x,y,yaw), 1 encoder body twist (vx,vy,yaw rate), 2 fused pose, 3 fused body twist, 4 estimated bias (ax,ay,gyro z). Units are metres, seconds and radians. On outage recovery, the last fused-from-encoder world transform is reapplied to the current encoder pose. Returns SAMPLE_NOT_AVAILABLE when the requested estimate is invalid. Match sequence with GET_FUSION_STATUS to associate timestamp/health; different sequences are different snapshots. At most 28 payload bytes so the framed reply fits Arduino Wire32. Does not reset state or start motion. Requires READY, exact v0.4.0 fusion capability and protocol 2.19. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, SENSOR_NOT_AVAILABLE, INTERNAL_ERROR, SAMPLE_NOT_AVAILABLE. Check lastError() after getters; setters return success. */
    fusion_vector get_fusion_vector(uint8_t kind);

    /** Read the volatile experiment mode and declared mounting yaw without changing state. Reset selects legacy encoder mode; SAVE_CONFIGURATION does not persist this experiment. Requires READY, exact v0.4.0 fusion capability and protocol 2.19. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, SENSOR_NOT_AVAILABLE, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    fusion_configuration get_fusion_configuration();

    /** Control volatile IMU-only relative tracking. START=0, RESET=2 and RECALIBRATE=3 are accepted from any state with stationary_mount_confirmed=1 and finite yaw in [-pi,pi]; all discard prior bias and pose and begin full cold stationary calibration at a new relative origin. The caller confirms true zero velocity, rigid level sensor-+Z-up mounting with IMU at the chassis reference point. Yaw is the angle of sensor +X in chassis XY, counterclockwise about chassis +Z. Sensor-to-chassis transform is Rz(yaw)*(x,y,z), axes forward/left/up. IMU stationarity checks cannot establish zero constant velocity. STOP=1 requires confirmation=0 and yaw=0 and stops integration. ACK confirms the action, not a valid estimate. Fault, expiry or interruption requires deliberate restart; no automatic recovery. Not saved to FRAM. Requires READY, exact model 1 hardware 0.4.0 with imu_tracking capability and protocol 2.20+. Independent of platform, encoders and fusion; never commands motion or changes control feedback. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, SENSOR_NOT_AVAILABLE, INTERNAL_ERROR, BOARD_IDLE, DRIVERS_ASLEEP. Check lastError() after getters; setters return success. */
    bool set_imu_tracking(uint8_t action, uint8_t stationary_mount_confirmed, double mounting_yaw_rad);

    /** Read IMU-only tracking state, fault, flags, sequence, joint sample-watermark timestamp, elapsed tracking time and separate accel/gyro ages in microseconds. UINT32_MAX age means unavailable. ESTIMATE_VALID is required for usable current estimates; timestamp and health are not physical accuracy guarantees. Reads can enforce expiry but never start tracking. Status and vector sequences must match to combine separate reads. Requires READY, exact model 1 hardware 0.4.0 with imu_tracking capability and protocol 2.20+. Independent of platform, encoders and fusion; never commands motion or changes control feedback. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, SENSOR_NOT_AVAILABLE, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    imu_tracking_status get_imu_tracking_status();

    /** Read a compact IMU-only vector selected by kind: POSE=0 (relative metres,metres,radians), TWIST=1 (body metres/second,metres/second,radians/second), BIAS=2 (accel-X,accel-Y metres/second squared and gyro-Z radians/second). SAMPLE_NOT_AVAILABLE means the requested estimate is invalid. Match sequence with GET_IMU_TRACKING_STATUS; never combine mismatched snapshots. Relative integration drifts and has no absolute position or heading reference. Requires READY, exact model 1 hardware 0.4.0 with imu_tracking capability and protocol 2.20+. Independent of platform, encoders and fusion; never commands motion or changes control feedback. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, SENSOR_NOT_AVAILABLE, INTERNAL_ERROR, SAMPLE_NOT_AVAILABLE. Check lastError() after getters; setters return success. */
    imu_tracking_vector get_imu_tracking_vector(uint8_t kind);

    /** Read mounting yaw and the actual bounded tracking horizon in microseconds. The development horizon is provisional; use this readback rather than assuming a fixed duration. Reset clears volatile tracking state; reconnect does not start or re-anchor tracking. Requires READY, exact model 1 hardware 0.4.0 with imu_tracking capability and protocol 2.20+. Independent of platform, encoders and fusion; never commands motion or changes control feedback. Errors: INVALID_LENGTH, INVALID_ARGUMENT, INIT_REQUIRED, CLOCK_NOT_READY, SENSOR_NOT_AVAILABLE, INTERNAL_ERROR. Check lastError() after getters; setters return success. */
    imu_tracking_configuration get_imu_tracking_configuration();
    bool set_imu_tracking(KinisiImuTrackingAction action, uint8_t confirmed, double yaw) { return set_imu_tracking(static_cast<uint8_t>(action), confirmed, yaw); }
    imu_tracking_vector get_imu_tracking_vector(KinisiImuTrackingVectorKind kind) { return get_imu_tracking_vector(static_cast<uint8_t>(kind)); }
    bool set_fusion_experiment(KinisiFusionMode mode, double yaw) { return set_fusion_experiment(static_cast<uint8_t>(mode), yaw); }
    fusion_vector get_fusion_vector(KinisiFusionVectorKind kind) { return get_fusion_vector(static_cast<uint8_t>(kind)); }
};
#endif
