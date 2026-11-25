// Generated from tools/commands.json; edit the schema or generator instead.
#ifndef KINISI_TYPES_H
#define KINISI_TYPES_H
#include <stdint.h>
static const uint8_t KINISI_GET_TEMPERATURE = 0x50;
static const uint8_t KINISI_GET_IMU = 0x51;
static const uint8_t KINISI_GET_MOTOR_CURRENT = 0x52;
static const uint8_t KINISI_GET_MOTOR_FAULTS = 0x53;
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
static const uint8_t KINISI_PING = 0x76;
static const uint8_t KINISI_SET_HEARTBEAT_CONFIG = 0x77;
static const uint8_t KINISI_GET_HEARTBEAT_CONFIG = 0x78;
/** @deprecated Retired since protocol 2.6.0. Use SUBSCRIBE. Kept for compatibility; eligible for removal in protocol 3.0.0. */
static const uint8_t KINISI_SUBSCRIBE_ODOMETRY = 0x79;
/** @deprecated Retired since protocol 2.6.0. Use UNSUBSCRIBE. Kept for compatibility; eligible for removal in protocol 3.0.0. */
static const uint8_t KINISI_UNSUBSCRIBE_ODOMETRY = 0x7a;
/** @deprecated Retired since protocol 2.6.0. Use TELEMETRY_EVENT. Kept for compatibility; eligible for removal in protocol 3.0.0. */
static const uint8_t KINISI_ENCODER_ODOMETRY_EVENT = 0x7b;
/** @deprecated Retired since protocol 2.6.0. Use TELEMETRY_EVENT. Kept for compatibility; eligible for removal in protocol 3.0.0. */
static const uint8_t KINISI_PLATFORM_ODOMETRY_EVENT = 0x7c;
static const uint8_t KINISI_POLL_TELEMETRY = 0x7d;
static const uint8_t KINISI_INITIALIZE_MOTOR_POSITION_CONTROLLER = 0x0C;
static const uint8_t KINISI_RESET_MOTOR_POSITION = 0x0D;
static const uint8_t KINISI_SET_MOTOR_POSITION = 0x0E;
static const uint8_t KINISI_GET_MOTOR_POSITION = 0x0F;
static const uint8_t KINISI_INITIALIZE_PLATFORM_POSITION_CONTROLLER = 0x4B;
static const uint8_t KINISI_RESET_PLATFORM_POSITION = 0x4C;
static const uint8_t KINISI_SET_PLATFORM_POSITION = 0x4D;
static const uint8_t KINISI_INITIALIZE_MOTOR_POSITION_PID_CONTROLLER = 0x10;
static const uint8_t KINISI_INITIALIZE_PLATFORM_POSITION_PID_CONTROLLER = 0x4E;
static const uint8_t KINISI_SUBSCRIBE = 0x80;
static const uint8_t KINISI_UNSUBSCRIBE = 0x81;
static const uint8_t KINISI_TELEMETRY_EVENT = 0x82;
static const uint8_t KINISI_SUBSCRIBE_WITH_MODE = 0x83;
static const uint8_t KINISI_CLEAR_MOTOR_FAULTS = 0x54;
static const uint8_t KINISI_GET_MOTOR_CURRENT_LIMIT = 0x55;
static const uint8_t KINISI_SET_MOTOR_CURRENT_LIMIT = 0x56;
static const uint8_t KINISI_SUBSCRIBE_CONFIGURED = 0x84;
static const uint8_t KINISI_GET_PWM_FREQUENCY = 0x57;
static const uint8_t KINISI_SET_PWM_FREQUENCY = 0x58;
static const uint8_t KINISI_SAVE_CONFIGURATION = 0x59;
static const uint8_t KINISI_GET_CONFIGURATION_FIELD = 0x5A;
static const uint8_t KINISI_SET_DRIVER_SLEEP = 0x5B;
static const uint8_t KINISI_GET_DRIVER_SLEEP = 0x5C;
static const uint8_t KINISI_SET_BOARD_IDLE = 0x5D;
static const uint8_t KINISI_GET_BOARD_IDLE = 0x5E;
static const uint8_t KINISI_SET_FUSION_EXPERIMENT = 0x5f;
static const uint8_t KINISI_GET_FUSION_STATUS = 0x60;
static const uint8_t KINISI_GET_FUSION_VECTOR = 0x61;
static const uint8_t KINISI_GET_FUSION_CONFIGURATION = 0x62;
static const uint8_t KINISI_SET_IMU_TRACKING = 0x63;
static const uint8_t KINISI_GET_IMU_TRACKING_STATUS = 0x64;
static const uint8_t KINISI_GET_IMU_TRACKING_VECTOR = 0x65;
static const uint8_t KINISI_GET_IMU_TRACKING_CONFIGURATION = 0x66;
static const uint8_t KINISI_SET_FEEDBACK_SOURCE = 0x67;
static const uint8_t KINISI_GET_FEEDBACK_SOURCE_STATUS = 0x68;
static const uint8_t KINISI_GET_FEEDBACK_SOURCE_VECTOR = 0x69;
static const uint8_t KINISI_SET_FEEDBACK_FUSION_MOUNT = 0x6a;
static const uint8_t KINISI_GET_FEEDBACK_FUSION_MOUNT = 0x6b;
static const uint8_t KINISI_GET_FEEDBACK_SOURCE_HEALTH = 0x6c;
static const uint8_t KINISI_GET_FEEDBACK_IMU_CHANNELS = 0x6d;
static const uint8_t KINISI_SET_IMU_MOUNT = 0x6e;
static const uint8_t KINISI_GET_IMU_MOUNT = 0x6f;
static const uint8_t KINISI_SET_BOARD_MOUNT = 0x85;
static const uint8_t KINISI_GET_BOARD_MOUNT = 0x86;
static const int32_t KINISI_MOUNT_READ_OFFSET_MAX_UM = 10031098L;

/** Shared controller error codes; see commands.json for per-command errors. */
enum class KinisiErrorCode : uint8_t {
    NONE = 0,
    INCOMPATIBLE_PROTOCOL = 1, // The requested protocol version is incompatible with this firmware.
    INVALID_ARGUMENT = 2, // A field is invalid, out of range, or inconsistent with the pending request.
    UNKNOWN_COMMAND = 3, // The received command ID is not accepted by the controller.
    INVALID_LENGTH = 4, // The complete message has an invalid header or payload length.
    MOTOR_OWNED = 5, // The motor belongs to a platform; use the corresponding platform command.
    INTERNAL_ERROR = 6, // The controller could not execute or encode the operation. During motor initialization, possible causes include missing motor power, a driver communication failure, or a faulty driver.
    CLOCK_NOT_READY = 7, // Initial clock setup is incomplete; wait for READY. Uptime mode also supports READY.
    TIME_SYNC_FAILED = 8, // Initial time sync exhausted its attempts without a valid sample.
    ODOMETRY_NOT_INITIALIZED = 9, // Odometry is not running; start odometry before requesting a measurement.
    SAMPLE_NOT_AVAILABLE = 10, // The requested measurement is not ready yet, including after initialization or reset; retry after an update.
    ENCODER_NOT_INITIALIZED = 11, // Initialize the encoder before reading it or starting encoder odometry.
    PLATFORM_NOT_INITIALIZED = 12, // Initialize a platform before performing this operation.
    MOTOR_NOT_INITIALIZED = 13, // Initialize the motor before setting its speed.
    CONTROLLER_NOT_INITIALIZED = 14, // Start or initialize the closed-loop controller before setting its target.
    INIT_REQUIRED = 15, // Send a valid INIT request before performing this connection-dependent operation.
    SENSOR_NOT_AVAILABLE = 16, // The selected sensor is not defined on this board.
    SUBSCRIPTION_LIMIT = 17, // This connection has no free subscription slots; unsubscribe an existing source first.
    SAMPLE_OUT_OF_RANGE = 18, // The sensor reading is saturated; no valid measurement can be returned.
    DRIVERS_ASLEEP = 19, // Driver sleep is active. Explicitly wake the drivers before requesting this operation.
    BOARD_IDLE = 20, // Board idle is active. Explicitly wake the board before changing driver sleep or starting a subscription.
    SOURCE_SWITCH_UNSAFE = 21, // Stop all motor outputs, disarm all controllers and wait for stationary wheel observations before changing the tracking preference.
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

/** Per-connection watchdog settings. */
struct heartbeat_config {
    bool enabled; ///< Enable monitoring; disabled initially for compatibility.
    uint32_t timeout_ms; ///< Monotonic receive timeout. Default 500 ms; send PING after at most timeout/5 of idle time (100 ms with the default).
};

/** Latest independent accelerometer and gyroscope samples in sensor axes. */
struct imu_sample {
    double acceleration_x; ///< X acceleration in m/s^2, including gravity.
    double acceleration_y; ///< Y acceleration in m/s^2, including gravity.
    double acceleration_z; ///< Z acceleration in m/s^2, including gravity.
    double angular_velocity_x; ///< X angular velocity in rad/s.
    double angular_velocity_y; ///< Y angular velocity in rad/s.
    double angular_velocity_z; ///< Z angular velocity in rad/s.
};

/** Timestamped temperature register reading in degrees Celsius. */
struct temperature_sample {
    uint64_t timestamp_us; ///< Sensor register-read completion timestamp in the selected clock domain; not a hardware conversion timestamp.
    uint8_t clock_mode; ///< 0 uptime, 1 Unix wall time.
    uint8_t clock_quality; ///< 1 valid, 2 stale.
    double celsius; ///< Temperature in degrees Celsius.
};

/** Timestamped IMU register reading. Acceleration and angular velocity are not synchronized with each other. */
struct imu_telemetry_sample {
    uint64_t timestamp_us; ///< Sensor register-read completion timestamp in the selected clock domain; not a hardware conversion timestamp.
    uint8_t clock_mode; ///< 0 uptime, 1 Unix wall time.
    uint8_t clock_quality; ///< 1 valid, 2 stale.
    double acceleration_x; ///< X acceleration in m/s^2, including gravity.
    double acceleration_y; ///< Y acceleration in m/s^2, including gravity.
    double acceleration_z; ///< Z acceleration in m/s^2, including gravity.
    double angular_velocity_x; ///< X angular velocity in rad/s.
    double angular_velocity_y; ///< Y angular velocity in rad/s.
    double angular_velocity_z; ///< Z angular velocity in rad/s.
};

/** Timestamped filtered motor-current estimate. */
struct motor_current_sample {
    uint64_t timestamp_us; ///< ADC read completion timestamp in the selected clock domain.
    uint8_t clock_mode; ///< 0 uptime, 1 Unix wall time.
    uint8_t clock_quality; ///< 1 valid, 2 stale.
    double amps; ///< Nonnegative filtered high-side current estimate in amperes.
};

/** Motor-driver diagnostic status. Ignore detailed flags when status_valid is false; a clear fault input alone does not establish driver health. */
struct motor_fault_status {
    bool sleep_requested; ///< The sleep control is asserted; reading never wakes the driver.
    bool status_valid; ///< Driver diagnostic registers were read successfully. Detailed flags are unknown when false.
    bool fault_active; ///< The fault input is asserted or a valid register reports a global fault.
    bool overtemperature_warning; ///< Driver temperature warning is reported.
    bool undervoltage; ///< Driver supply undervoltage is reported; this does not prove a disconnected supply.
    bool charge_pump_undervoltage; ///< Driver charge-pump undervoltage is reported.
    bool overcurrent; ///< An overcurrent fault is reported.
    bool overtemperature_shutdown; ///< A thermal shutdown fault is reported.
    bool open_load; ///< The configured open-load detector reports a fault; detection depends on operating conditions.
    bool current_regulation; ///< At least one output reports current regulation; this is not an overcurrent fault.
};

/** Timestamped motor-driver diagnostic status. Timestamp marks completion of the read; faults can be latched rather than new events. */
struct motor_fault_sample {
    uint64_t timestamp_us; ///< ADC read completion timestamp in the selected clock domain.
    uint8_t clock_mode; ///< 0 uptime, 1 Unix wall time.
    uint8_t clock_quality; ///< 1 valid, 2 stale.
    bool sleep_requested; ///< The sleep control is asserted; reading never wakes the driver.
    bool status_valid; ///< Driver diagnostic registers were read successfully. Detailed flags are unknown when false.
    bool fault_active; ///< The fault input is asserted or a valid register reports a global fault.
    bool overtemperature_warning; ///< Driver temperature warning is reported.
    bool undervoltage; ///< Driver supply undervoltage is reported; this does not prove a disconnected supply.
    bool charge_pump_undervoltage; ///< Driver charge-pump undervoltage is reported.
    bool overcurrent; ///< An overcurrent fault is reported.
    bool overtemperature_shutdown; ///< A thermal shutdown fault is reported.
    bool open_load; ///< The configured open-load detector reports a fault; detection depends on operating conditions.
    bool current_regulation; ///< At least one output reports current regulation; this is not an overcurrent fault.
};

/** Applied nominal hardware current-regulation setting. */
struct motor_current_limit {
    bool enabled; ///< Current regulation is enabled on both outputs.
    uint16_t threshold_ma; ///< Nominal threshold in milliamps.
};

/** Volatile v0.4 fusion experiment diagnostics; see command descriptions. */
struct fusion_status {
    uint8_t mode; ///< Selected fusion_modes value.
    uint8_t health; ///< Current fusion_health value.
    uint8_t fault; ///< Current fusion_faults reason, or NONE.
    uint8_t flags; ///< Bitmask of fusion_flags; ESTIMATE_VALID is required before using estimates.
    uint32_t sequence; ///< Snapshot sequence; compare with each vector before associating this status.
    uint64_t timestamp_us; ///< Monotonic board microseconds of this estimate snapshot; not wall time.
    uint32_t accel_age_us; ///< UINT32_MAX means unavailable.
    uint32_t gyro_age_us; ///< UINT32_MAX means unavailable.
    uint32_t encoder_age_us; ///< UINT32_MAX means unavailable.
};

/** Volatile v0.4 fusion experiment diagnostics; see command descriptions. */
struct fusion_vector {
    uint32_t sequence; ///< Match GET_FUSION_STATUS sequence before associating its timestamp or health.
    double x; ///< Pose X in metres, twist X in metres per second, or X accelerometer bias in metres per second squared, selected by kind.
    double y; ///< Pose Y in metres, twist Y in metres per second, or Y accelerometer bias in metres per second squared, selected by kind.
    double z; ///< Pose yaw in radians, twist yaw rate in radians per second, or Z gyro bias in radians per second, selected by kind.
};

/** Volatile v0.4 fusion experiment diagnostics; see command descriptions. */
struct fusion_configuration {
    uint8_t mode; ///< Selected fusion_modes value; boot default is LEGACY_ENCODER.
    double mounting_yaw_rad; ///< Angle of board +X in the chassis XY plane, counterclockwise about chassis +Z; finite radians in [-pi, pi].
};

/** Volatile IMU-only relative tracking diagnostics. */
struct imu_tracking_status {
    uint8_t state; ///< ImuTrackingState value.
    uint8_t fault; ///< ImuTrackingFault value.
    uint8_t flags; ///< ImuTrackingFlag bitmask.
    uint8_t reserved; ///< Reserved; must be zero.
    uint32_t sequence; ///< Snapshot sequence; match vector reads.
    uint64_t timestamp_us; ///< Last eligible monotonic joint accel/gyro source watermark; no published watermark reaches origin + window_us. Zero before the first estimate.
    uint32_t elapsed_us; ///< Wall elapsed microseconds since the source origin, capped at window_us; distinct from the retained source watermark. Frozen on fault or expiry; STOP clears it.
    uint32_t accel_age_us; ///< Accel age in microseconds; UINT32_MAX if unavailable.
    uint32_t gyro_age_us; ///< Gyro age in microseconds; UINT32_MAX if unavailable.
};

/** Volatile IMU-only relative tracking diagnostics. */
struct imu_tracking_vector {
    uint32_t sequence; ///< Match status sequence before associating health and time.
    double x; ///< Pose X, body velocity X, or acceleration bias X, selected by kind.
    double y; ///< Pose Y, body velocity Y, or acceleration bias Y, selected by kind.
    double z; ///< Pose yaw, body yaw rate, or gyro Z bias, selected by kind.
};

/** Volatile IMU-only relative tracking diagnostics. */
struct imu_tracking_configuration {
    double mounting_yaw_rad; ///< Declared supported mounting yaw in radians.
    uint32_t window_us; ///< Actual source tracking horizon in microseconds; 1000000 for this v0.4 release. Eligible source times are [origin, origin + window_us).
};

/** Board-wide selected tracking telemetry status. */
struct feedback_source_status {
    uint8_t requested; ///< Requested FeedbackSource.
    uint8_t active; ///< Active FeedbackSource, NONE while invalid.
    uint8_t valid; ///< One when a current source snapshot is available, otherwise zero.
    uint8_t reason; ///< FeedbackSourceReason.
    uint32_t sequence; ///< Underlying sample sequence when valid; zero when invalid.
    uint64_t timestamp_us; ///< Underlying monotonic board sample time; zero when invalid.
    uint32_t origin_id; ///< Board-local frame generation. Discard prior coordinates on change or board reboot.
    uint8_t source_detail; ///< Native IMU tracking or fusion fault code for the requested source; zero otherwise.
    uint32_t age_us; ///< Board-clock sample age in microseconds; UINT32_MAX when unavailable.
    uint16_t max_age_ms; ///< Maximum eligible captured sample age in milliseconds for this source, including the configured encoder period. Compare updated original sample age with the smaller of its captured and current limits.
};

/** Selected source vector in the selector pose frame or native body twist frame. */
struct feedback_source_vector {
    uint32_t sequence; ///< Exact producer sequence captured by status and requested by the vector command.
    double x; ///< X coordinate or body X velocity.
    double y; ///< Y coordinate or body Y velocity.
    double z; ///< Continuous relative yaw or body yaw rate.
};

/** Passive tracking mount, independent of chassis control. */
struct feedback_fusion_mount {
    uint8_t confirmed; ///< One after an explicit mount declaration this boot.
    double mounting_yaw_rad; ///< Angle of board +X in chassis XY, counterclockwise about +Z in radians. A centered, level board with +Z up is required only for a yaw-only installation.
};

/** Read producer health independently of selector and control. ENCODER is the running platform encoder batch, IMU_ONLY is physical BMI088 pair availability independent of the one-second estimate, FUSED is the passive fused estimate. Disagreement indicates conflicting observations without assigning sensor blame. HEALTHY means directly usable data. Comparability is separate. Historical provenance remains readable during failure. Reconnect reads board state. */
struct feedback_source_health {
    uint8_t source; ///< Echo of FeedbackSource request.
    uint8_t health; ///< FeedbackSourceHealthState.
    uint8_t reason; ///< FeedbackSourceReason for current producer availability.
    uint8_t detail; ///< FusionFault explanation if present.
    uint32_t flags; ///< FeedbackSourceHealthFlag bit mask.
    uint32_t sequence; ///< Sequence of the last producer publication or observed distinct raw IMU pair, wrapping modulo 2^32; use PROVENANCE for presence.
    uint64_t timestamp_us; ///< Last usable source time in board uptime microseconds. For the BMI088 pair this is its older die timestamp.
    uint32_t age_us; ///< Age of that timestamp in board uptime microseconds, UINT32_MAX if absent, future or too old to represent.
    uint32_t max_age_us; ///< Source freshness budget in microseconds.
};

/** Read independent last observed accelerometer and gyro acquisition times in board uptime microseconds. A timestamp of zero and age UINT32_MAX indicate no observed acquisition. These are physical source observations, independent of the finite IMU-only tracking window. A pair must meet the 35000 us age and 15000 us skew limits. */
struct feedback_imu_channels {
    uint64_t accel_timestamp_us; ///< Last observed accelerometer acquisition time in board uptime microseconds.
    uint64_t gyro_timestamp_us; ///< Last observed gyro acquisition time in board uptime microseconds.
    uint32_t accel_age_us; ///< Board-derived acquisition age in microseconds, UINT32_MAX if unknown.
    uint32_t gyro_age_us; ///< Board-derived acquisition age in microseconds, UINT32_MAX if unknown.
};

/** Signed board orientation and displacement to the chip in chassis axes. */
struct imu_mount_configuration {
    int16_t roll_cdeg; ///< Board-to-chassis roll in signed hundredths of a degree.
    int16_t pitch_cdeg; ///< Board-to-chassis pitch in signed hundredths of a degree.
    int16_t yaw_cdeg; ///< Board-to-chassis yaw in signed hundredths of a degree.
    int32_t x_um; ///< Signed displacement from the tracking origin to the BMI088 chip along chassis +X (forward), in micrometres. A position derived from the other reference can exceed its setter's 10 m input range.
    int32_t y_um; ///< Signed displacement from the tracking origin to the BMI088 chip along chassis +Y (left), in micrometres. A position derived from the other reference can exceed its setter's 10 m input range.
    int32_t z_um; ///< Signed displacement from the tracking origin to the BMI088 chip along chassis +Z (up), in micrometres. A position derived from the other reference can exceed its setter's 10 m input range.
    bool configured; ///< True after explicit mount setup or restore. False selects legacy per-mode yaw.
};

/** Applied orientation and chassis-to-PCB underside-center displacement. */
struct board_mount_configuration {
    int16_t roll_cdeg; ///< Board-to-chassis roll in signed hundredths of a degree.
    int16_t pitch_cdeg; ///< Board-to-chassis pitch in signed hundredths of a degree.
    int16_t yaw_cdeg; ///< Board-to-chassis yaw in signed hundredths of a degree.
    int32_t x_um; ///< Signed displacement from the tracking origin to the geometric center of the PCB underside along chassis +X (forward), in micrometres. A position derived from the other reference can exceed its setter's 10 m input range.
    int32_t y_um; ///< Signed displacement from the tracking origin to the geometric center of the PCB underside along chassis +Y (left), in micrometres. A position derived from the other reference can exceed its setter's 10 m input range.
    int32_t z_um; ///< Signed displacement from the tracking origin to the geometric center of the PCB underside along chassis +Z (up), in micrometres. A position derived from the other reference can exceed its setter's 10 m input range.
    bool configured; ///< True after explicit mount setup or restore. False selects legacy per-mode yaw.
};
enum class KinisiSubscriptionTrigger : uint8_t {
    PERIODIC = 0,
    ON_EVENT = 1,
};
enum class KinisiSubscriptionDelivery : uint8_t {
    ALL = 0,
    ON_CHANGE = 1,
};
enum class KinisiFusionMode : uint8_t {
    LEGACY_ENCODER = 0,
    ENCODER_CORRECTION = 1,
    FUSED = 2,
};
enum class KinisiFusionHealth : uint8_t {
    DISABLED = 0,
    WAITING = 1,
    HEALTHY = 2,
    FAULT = 3,
};
enum class KinisiFusionFault : uint8_t {
    NONE = 0,
    MOUNT_INVALID = 1,
    ENCODER_UNAVAILABLE = 2,
    IMU_UNAVAILABLE = 3,
    STALE = 4,
    TIMESTAMP = 5,
    SATURATION = 6,
    BIAS_NOT_READY = 7,
    DISCONTINUITY = 8,
    INHIBITED = 9,
};
enum class KinisiFusionFlag : uint8_t {
    ESTIMATE_VALID = 1,
    BIAS_READY = 2,
    FRESH_TARGET_REQUIRED = 4,
    CORRECTION_ACTIVE = 8,
};
enum class KinisiFusionVectorKind : uint8_t {
    ENCODER_POSE = 0,
    ENCODER_TWIST = 1,
    FUSED_POSE = 2,
    FUSED_TWIST = 3,
    BIAS = 4,
};
enum class KinisiImuTrackingAction : uint8_t {
    START = 0,
    STOP = 1,
    RESET = 2,
    RECALIBRATE = 3,
};
enum class KinisiImuTrackingState : uint8_t {
    STOPPED = 0,
    CALIBRATING = 1,
    TRACKING = 2,
    FAULT = 3,
    EXPIRED = 4,
};
enum class KinisiImuTrackingFault : uint8_t {
    NONE = 0,
    NOT_STATIONARY = 1,
    IMU_UNAVAILABLE = 2,
    STALE = 3,
    TIMESTAMP = 4,
    SATURATION = 5,
    BIAS_NOT_READY = 6,
    WINDOW_EXPIRED = 7,
    INHIBITED = 8,
    MOUNT_INVALID = 9,
};
enum class KinisiImuTrackingFlag : uint8_t {
    ESTIMATE_VALID = 1,
    BIAS_READY = 2,
    RESTART_REQUIRED = 4,
    MOUNT_CONFIRMED = 8,
};
enum class KinisiImuTrackingVectorKind : uint8_t {
    POSE = 0,
    TWIST = 1,
    BIAS = 2,
};
enum class KinisiFeedbackSource : uint8_t {
    ENCODER = 0,
    IMU_ONLY = 1,
    FUSED = 2,
    NONE = 255,
};
enum class KinisiFeedbackSourceReason : uint8_t {
    OK = 0,
    NOT_STARTED = 1,
    CALIBRATING = 2,
    UNAVAILABLE = 3,
    STALE = 4,
    EXPIRED = 5,
    FAULT = 6,
    INHIBITED = 7,
    WAITING_SAMPLE = 8,
    MOUNT_REQUIRED = 9,
};
enum class KinisiFeedbackSourceVectorKind : uint8_t {
    POSE = 0,
    TWIST = 1,
};
enum class KinisiFeedbackSourceHealthState : uint8_t {
    HEALTHY = 0,
    SUSPECT = 1,
    UNAVAILABLE = 2,
    INDETERMINATE = 3,
};
enum class KinisiFeedbackSourceHealthFlag : uint32_t {
    PROVENANCE = 1,
    FRESH = 2,
    DISAGREEMENT = 4,
    COMPARISON_AVAILABLE = 8,
    RECOVERING = 16,
    RESTART_REQUIRED = 32,
    TIMESTAMP_FAULT = 64,
    SATURATION = 128,
};
enum class KinisiPwmFrequency : uint32_t {
    KHZ_10 = 10000UL,
    KHZ_20 = 20000UL,
    KHZ_25 = 25000UL,
    KHZ_50 = 50000UL,
    KHZ_100 = 100000UL,
};

/** Delivery behavior for generic subscriptions. */
enum class KinisiSubscriptionMode : uint8_t {
    PERIODIC = 0,
    ON_CHANGE = 1,
};

/** Stable wire identifiers for telemetry sources. */
enum class KinisiTopic : uint8_t {
    ENCODER_ODOMETRY = 1,
    PLATFORM_ODOMETRY = 2,
    TEMPERATURE = 3,
    IMU = 4,
    MOTOR_CURRENT = 5,
    MOTOR_FAULTS = 6,
};

/** Only the union member selected by topic is valid. */
struct KinisiTelemetrySample {
    KinisiTopic topic;
    uint8_t index;
    union {
        encoder_odometry_sample encoder_odometry;
        platform_odometry_sample platform_odometry;
        temperature_sample temperature;
        imu_telemetry_sample imu;
        motor_current_sample motor_current;
        motor_fault_sample motor_faults;
    } data;
};
bool kinisiTopicInfo(KinisiTopic topic, uint8_t index, uint8_t& size, uint32_t& minimum_ms);
bool kinisiDecodeTelemetry(KinisiTopic topic, uint8_t index, const uint8_t* payload, uint8_t size, KinisiTelemetrySample& sample);
bool kinisiPwmFrequencyValid(uint32_t frequency);
#endif
bool kinisiTopicEventSupported(KinisiTopic topic);
enum class KinisiFeedbackRead : uint8_t { VALID, UNAVAILABLE, CHANGED, ERROR };
