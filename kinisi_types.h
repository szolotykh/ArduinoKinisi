// Generated from tools/commands.json; edit the schema or generator instead.
#ifndef KINISI_TYPES_H
#define KINISI_TYPES_H
#include <stdint.h>
static const uint8_t KINISI_INITIALIZE_MOTOR = 0x01;
static const uint8_t KINISI_SET_MOTOR_SPEED = 0x02;
static const uint8_t KINISI_STOP_MOTOR = 0x03;
static const uint8_t KINISI_BRAKE_MOTOR = 0x04;
static const uint8_t KINISI_INITIALIZE_MOTOR_CONTROLLER = 0x05;
static const uint8_t KINISI_SET_MOTOR_TARGET_SPEED = 0x06;
static const uint8_t KINISI_RESET_MOTOR_CONTROLLER = 0x07;
static const uint8_t KINISI_GET_MOTOR_CONTROLLER_STATE = 0x08;
static const uint8_t KINISI_DELETE_MOTOR_CONTROLLER = 0x09;
static const uint8_t KINISI_SET_CONTROLLER_FREQUENCY = 0x0A;
static const uint8_t KINISI_GET_CONTROLLER_FREQUENCY = 0x0B;
static const uint8_t KINISI_INITIALIZE_ENCODER = 0x11;
static const uint8_t KINISI_GET_ENCODER_VALUE = 0x12;
static const uint8_t KINISI_START_ENCODER_ODOMETRY = 0x13;
static const uint8_t KINISI_RESET_ENCODER_ODOMETRY = 0x14;
static const uint8_t KINISI_STOP_ENCODER_ODOMETRY = 0x15;
static const uint8_t KINISI_GET_ENCODER_ODOMETRY = 0x16;
static const uint8_t KINISI_SET_ODOMETRY_FREQUENCY = 0x17;
static const uint8_t KINISI_GET_ODOMETRY_FREQUENCY = 0x18;
static const uint8_t KINISI_INITIALIZE_GPIO_PIN = 0x20;
static const uint8_t KINISI_SET_GPIO_PIN_STATE = 0x21;
static const uint8_t KINISI_GET_GPIO_PIN_STATE = 0x22;
static const uint8_t KINISI_TOGGLE_GPIO_PIN_STATE = 0x23;
static const uint8_t KINISI_SET_STATUS_LED_STATE = 0x25;
static const uint8_t KINISI_TOGGLE_STATUS_LED_STATE = 0x26;
static const uint8_t KINISI_INITIALIZE_MECANUM_PLATFORM = 0x30;
static const uint8_t KINISI_INITIALIZE_OMNI_PLATFORM = 0x31;
static const uint8_t KINISI_INITIALIZE_DIFFERENTIAL_PLATFORM = 0x32;
static const uint8_t KINISI_SET_PLATFORM_VELOCITY = 0x40;
static const uint8_t KINISI_START_PLATFORM_CONTROLLER = 0x41;
static const uint8_t KINISI_SET_PLATFORM_TARGET_VELOCITY = 0x42;
static const uint8_t KINISI_GET_PLATFORM_CURRENT_VELOCITY = 0x43;
static const uint8_t KINISI_STOP_PLATFORM_CONTROLLER = 0x44;
static const uint8_t KINISI_START_PLATFORM_ODOMETRY = 0x45;
static const uint8_t KINISI_RESET_PLATFORM_ODOMETRY = 0x46;
static const uint8_t KINISI_STOP_PLATFORM_ODOMETRY = 0x47;
static const uint8_t KINISI_GET_PLATFORM_ODOMETRY = 0x48;
static const uint8_t KINISI_BRAKE_PLATFORM = 0x49;
static const uint8_t KINISI_COAST_PLATFORM = 0x4A;
static const uint8_t KINISI_INIT = 0x70;
static const uint8_t KINISI_ERROR = 0x7F;
static const uint8_t KINISI_TIME_SYNC_REQUEST = 0x71;
static const uint8_t KINISI_TIME_SYNC_RESPONSE = 0x72;
static const uint8_t KINISI_READY = 0x73;
static const uint8_t KINISI_SET_TIME_SYNC_INTERVAL = 0x74;
static const uint8_t KINISI_GET_TIME_STATUS = 0x75;

/** Shared controller error codes; see commands.json for per-command errors. */
enum class KinisiErrorCode : uint8_t {
    NONE = 0,
    INCOMPATIBLE_PROTOCOL = 1, // The requested protocol version is incompatible with this firmware.
    INVALID_ARGUMENT = 2, // A field is invalid, out of range, or inconsistent with the pending request.
    UNKNOWN_COMMAND = 3, // The received command ID is not accepted by the controller.
    INVALID_LENGTH = 4, // The complete message has an invalid header or payload length.
    MOTOR_OWNED = 5, // The motor belongs to a platform; use the corresponding platform command.
    INTERNAL_ERROR = 6, // The controller could not execute or encode the operation.
    CLOCK_NOT_READY = 7, // Initial clock setup is incomplete; wait for READY. Uptime mode also supports READY.
    TIME_SYNC_FAILED = 8, // Initial time sync exhausted its attempts without a valid sample.
    ODOMETRY_NOT_INITIALIZED = 9, // Odometry is not running; start odometry before requesting a measurement.
    SAMPLE_NOT_AVAILABLE = 10, // Odometry is running but has no measurement yet, including immediately after reset; retry after an update.
    ENCODER_NOT_INITIALIZED = 11, // Initialize the encoder before reading it or starting encoder odometry.
    PLATFORM_NOT_INITIALIZED = 12, // Initialize a platform before performing this operation.
    MOTOR_NOT_INITIALIZED = 13, // Initialize the motor before setting its speed.
    CONTROLLER_NOT_INITIALIZED = 14, // Start or initialize the closed-loop controller before setting its target.
    INIT_REQUIRED = 15, // Send a valid INIT request before performing this connection-dependent operation.
};

/** The state of the controller for the specified motor. */
struct motor_controller_state {
    int8_t motor_index; ///< Index of the motor with the controller.
    double kp; ///< Proportional constant of PID
    double ki; ///< Integral constant of PID
    double kd; ///< Derivative constant of PID
    double target_speed; ///< The target speed of the motor.
    double current_speed; ///< The current speed of the motor.
    double error; ///< The error of the motor.
    double output; ///< The output of the motor.
};

/** The velocity of the platform in meters per second. */
struct platform_velocity {
    double x; ///< X component of platform velocity in meters per second
    double y; ///< Y component of platform velocity in meters per second
    double t; ///< Theta component of platform velocity in radians per second
};

/** The odometry of the platform in meters and radians. */
struct platform_odometry {
    double x; ///< X component of platform odometry in meters
    double y; ///< Y component of platform odometry in meters
    double t; ///< Theta component of platform odometry in radians
};

/** Packed 15-byte INIT response; firmware build ID is a Git revision, not a semantic release version. */
struct init_response {
    uint8_t board_model; ///< 1 Kinisi motor controller.
    uint8_t board_major; ///< Hardware revision major.
    uint8_t board_minor; ///< Hardware revision minor.
    uint8_t board_patch; ///< Hardware revision patch.
    uint8_t protocol_major; ///< Protocol version major.
    uint8_t protocol_minor; ///< Protocol version minor.
    uint8_t protocol_patch; ///< Protocol version patch.
    uint32_t firmware_build_high; ///< First eight hexadecimal digits of the firmware Git commit.
    uint32_t firmware_build_low; ///< Next eight hexadecimal digits of the firmware Git commit.
};

/** Per-connection clock status. */
struct time_status {
    uint8_t clock_mode; ///< 0 uptime, 1 Unix wall time.
    uint8_t clock_quality; ///< 0 unready, 1 valid, 2 stale.
    uint32_t interval_ms; ///< Sync interval.
    uint64_t last_sync_age_us; ///< Elapsed since successful sync; zero in uptime or unready mode.
};

/** Odometry state with controller-captured timestamp and clock metadata. */
struct encoder_odometry_sample {
    uint64_t timestamp_us; ///< Measurement timestamp in selected clock domain.
    uint8_t clock_mode; ///< 0 uptime, 1 Unix wall time.
    uint8_t clock_quality; ///< 1 valid, 2 stale.
    double angle; ///< Encoder odometry radians.
};

/** Odometry state with controller-captured timestamp and clock metadata. */
struct platform_odometry_sample {
    uint64_t timestamp_us; ///< Measurement timestamp in selected clock domain.
    uint8_t clock_mode; ///< 0 uptime, 1 Unix wall time.
    uint8_t clock_quality; ///< 1 valid, 2 stale.
    double x; ///< X meters.
    double y; ///< Y meters.
    double t; ///< Heading radians.
};
#endif
