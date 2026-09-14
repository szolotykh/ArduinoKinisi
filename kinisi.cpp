// Generated from tools/commands.json; edit the schema or generator instead.
#include "kinisi.h"
#include "kinisi_codec.h"

/** Encode INITIALIZE_MOTOR, match its reply, and decode the payload. */
bool KinisiController::initialize_motor(uint8_t motor_index, bool is_reversed) {
    uint8_t payload[2] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(motor_index), 1);
    kinisi_codec::writeUnsigned(payload + 1, static_cast<uint64_t>(is_reversed), 1);
    return request(KINISI_INITIALIZE_MOTOR, payload, 2, nullptr, 0);
}

/** Encode SET_MOTOR_SPEED, match its reply, and decode the payload. */
bool KinisiController::set_motor_speed(uint8_t motor_index, double pwm) {
    uint8_t payload[9] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(motor_index), 1);
    kinisi_codec::writeDouble(payload + 1, pwm);
    return request(KINISI_SET_MOTOR_SPEED, payload, 9, nullptr, 0);
}

/** Encode STOP_MOTOR, match its reply, and decode the payload. */
bool KinisiController::stop_motor(uint8_t motor_index) {
    uint8_t payload[1] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(motor_index), 1);
    return request(KINISI_STOP_MOTOR, payload, 1, nullptr, 0);
}

/** Encode BRAKE_MOTOR, match its reply, and decode the payload. */
bool KinisiController::brake_motor(uint8_t motor_index) {
    uint8_t payload[1] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(motor_index), 1);
    return request(KINISI_BRAKE_MOTOR, payload, 1, nullptr, 0);
}

/** Encode INITIALIZE_MOTOR_CONTROLLER, match its reply, and decode the payload. */
bool KinisiController::initialize_motor_controller(uint8_t motor_index, bool is_reversed, uint8_t encoder_index, bool is_encoder_reversed, double encoder_resolution, double kp, double ki, double kd, double integral_limit) {
    uint8_t payload[44] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(motor_index), 1);
    kinisi_codec::writeUnsigned(payload + 1, static_cast<uint64_t>(is_reversed), 1);
    kinisi_codec::writeUnsigned(payload + 2, static_cast<uint64_t>(encoder_index), 1);
    kinisi_codec::writeUnsigned(payload + 3, static_cast<uint64_t>(is_encoder_reversed), 1);
    kinisi_codec::writeDouble(payload + 4, encoder_resolution);
    kinisi_codec::writeDouble(payload + 12, kp);
    kinisi_codec::writeDouble(payload + 20, ki);
    kinisi_codec::writeDouble(payload + 28, kd);
    kinisi_codec::writeDouble(payload + 36, integral_limit);
    return request(KINISI_INITIALIZE_MOTOR_CONTROLLER, payload, 44, nullptr, 0);
}

/** Encode SET_MOTOR_TARGET_SPEED, match its reply, and decode the payload. */
bool KinisiController::set_motor_target_speed(uint8_t motor_index, double speed) {
    uint8_t payload[9] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(motor_index), 1);
    kinisi_codec::writeDouble(payload + 1, speed);
    return request(KINISI_SET_MOTOR_TARGET_SPEED, payload, 9, nullptr, 0);
}

/** Encode RESET_MOTOR_CONTROLLER, match its reply, and decode the payload. */
bool KinisiController::reset_motor_controller(uint8_t motor_index) {
    uint8_t payload[1] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(motor_index), 1);
    return request(KINISI_RESET_MOTOR_CONTROLLER, payload, 1, nullptr, 0);
}

/** Encode GET_MOTOR_CONTROLLER_STATE, match its reply, and decode the payload. */
motor_controller_state KinisiController::get_motor_controller_state(uint8_t motor_index) {
    uint8_t payload[1] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(motor_index), 1);
    motor_controller_state result = {};
    uint8_t response[57] = {0};
    if (!request(KINISI_GET_MOTOR_CONTROLLER_STATE, payload, 1, response, 57)) return result;
    result.motor_index = static_cast<int8_t>(kinisi_codec::readUnsigned(response + 0, 1));
    result.kp = kinisi_codec::readDouble(response + 1);
    result.ki = kinisi_codec::readDouble(response + 9);
    result.kd = kinisi_codec::readDouble(response + 17);
    result.target_speed = kinisi_codec::readDouble(response + 25);
    result.current_speed = kinisi_codec::readDouble(response + 33);
    result.error = kinisi_codec::readDouble(response + 41);
    result.output = kinisi_codec::readDouble(response + 49);
    return result;
}

/** Encode DELETE_MOTOR_CONTROLLER, match its reply, and decode the payload. */
bool KinisiController::delete_motor_controller(uint8_t motor_index) {
    uint8_t payload[1] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(motor_index), 1);
    return request(KINISI_DELETE_MOTOR_CONTROLLER, payload, 1, nullptr, 0);
}

/** Encode SET_CONTROLLER_FREQUENCY, match its reply, and decode the payload. */
bool KinisiController::set_controller_frequency(uint16_t frequency) {
    uint8_t payload[2] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(frequency), 2);
    return request(KINISI_SET_CONTROLLER_FREQUENCY, payload, 2, nullptr, 0);
}

/** Encode GET_CONTROLLER_FREQUENCY, match its reply, and decode the payload. */
uint16_t KinisiController::get_controller_frequency() {
    uint8_t payload[1] = {0};
    uint16_t result = {};
    uint8_t response[2] = {0};
    if (!request(KINISI_GET_CONTROLLER_FREQUENCY, payload, 0, response, 2)) return result;
    result = static_cast<uint16_t>(kinisi_codec::readUnsigned(response, 2));
    return result;
}

/** Encode INITIALIZE_ENCODER, match its reply, and decode the payload. */
bool KinisiController::initialize_encoder(uint8_t encoder_index, double encoder_resolution, bool is_reversed) {
    uint8_t payload[10] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(encoder_index), 1);
    kinisi_codec::writeDouble(payload + 1, encoder_resolution);
    kinisi_codec::writeUnsigned(payload + 9, static_cast<uint64_t>(is_reversed), 1);
    return request(KINISI_INITIALIZE_ENCODER, payload, 10, nullptr, 0);
}

/** Encode GET_ENCODER_VALUE, match its reply, and decode the payload. */
uint16_t KinisiController::get_encoder_value(uint8_t encoder_index) {
    uint8_t payload[1] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(encoder_index), 1);
    uint16_t result = {};
    uint8_t response[2] = {0};
    if (!request(KINISI_GET_ENCODER_VALUE, payload, 1, response, 2)) return result;
    result = static_cast<uint16_t>(kinisi_codec::readUnsigned(response, 2));
    return result;
}

/** Encode START_ENCODER_ODOMETRY, match its reply, and decode the payload. */
bool KinisiController::start_encoder_odometry(uint8_t encoder_index) {
    uint8_t payload[1] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(encoder_index), 1);
    return request(KINISI_START_ENCODER_ODOMETRY, payload, 1, nullptr, 0);
}

/** Encode RESET_ENCODER_ODOMETRY, match its reply, and decode the payload. */
bool KinisiController::reset_encoder_odometry(uint8_t encoder_index) {
    uint8_t payload[1] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(encoder_index), 1);
    return request(KINISI_RESET_ENCODER_ODOMETRY, payload, 1, nullptr, 0);
}

/** Encode STOP_ENCODER_ODOMETRY, match its reply, and decode the payload. */
bool KinisiController::stop_encoder_odometry(uint8_t encoder_index) {
    uint8_t payload[1] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(encoder_index), 1);
    return request(KINISI_STOP_ENCODER_ODOMETRY, payload, 1, nullptr, 0);
}

/** Encode GET_ENCODER_ODOMETRY, match its reply, and decode the payload. */
encoder_odometry_sample KinisiController::get_encoder_odometry(uint8_t encoder_index) {
    uint8_t payload[1] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(encoder_index), 1);
    encoder_odometry_sample result = {};
    uint8_t response[18] = {0};
    if (!request(KINISI_GET_ENCODER_ODOMETRY, payload, 1, response, 18)) return result;
    result.timestamp_us = static_cast<uint64_t>(kinisi_codec::readUnsigned(response + 0, 8));
    result.clock_mode = static_cast<uint8_t>(kinisi_codec::readUnsigned(response + 8, 1));
    result.clock_quality = static_cast<uint8_t>(kinisi_codec::readUnsigned(response + 9, 1));
    result.angle = kinisi_codec::readDouble(response + 10);
    return result;
}

/** Encode SET_ODOMETRY_FREQUENCY, match its reply, and decode the payload. */
bool KinisiController::set_odometry_frequency(uint16_t frequency) {
    uint8_t payload[2] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(frequency), 2);
    return request(KINISI_SET_ODOMETRY_FREQUENCY, payload, 2, nullptr, 0);
}

/** Encode GET_ODOMETRY_FREQUENCY, match its reply, and decode the payload. */
uint16_t KinisiController::get_odometry_frequency() {
    uint8_t payload[1] = {0};
    uint16_t result = {};
    uint8_t response[2] = {0};
    if (!request(KINISI_GET_ODOMETRY_FREQUENCY, payload, 0, response, 2)) return result;
    result = static_cast<uint16_t>(kinisi_codec::readUnsigned(response, 2));
    return result;
}

/** Encode INITIALIZE_GPIO_PIN, match its reply, and decode the payload. */
bool KinisiController::initialize_gpio_pin(uint8_t pin_number, uint8_t mode) {
    uint8_t payload[2] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(pin_number), 1);
    kinisi_codec::writeUnsigned(payload + 1, static_cast<uint64_t>(mode), 1);
    return request(KINISI_INITIALIZE_GPIO_PIN, payload, 2, nullptr, 0);
}

/** Encode SET_GPIO_PIN_STATE, match its reply, and decode the payload. */
bool KinisiController::set_gpio_pin_state(uint8_t pin_number, uint8_t state) {
    uint8_t payload[2] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(pin_number), 1);
    kinisi_codec::writeUnsigned(payload + 1, static_cast<uint64_t>(state), 1);
    return request(KINISI_SET_GPIO_PIN_STATE, payload, 2, nullptr, 0);
}

/** Encode GET_GPIO_PIN_STATE, match its reply, and decode the payload. */
uint8_t KinisiController::get_gpio_pin_state(uint8_t pin_number) {
    uint8_t payload[1] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(pin_number), 1);
    uint8_t result = {};
    uint8_t response[1] = {0};
    if (!request(KINISI_GET_GPIO_PIN_STATE, payload, 1, response, 1)) return result;
    result = static_cast<uint8_t>(kinisi_codec::readUnsigned(response, 1));
    return result;
}

/** Encode TOGGLE_GPIO_PIN_STATE, match its reply, and decode the payload. */
bool KinisiController::toggle_gpio_pin_state(uint8_t pin_number) {
    uint8_t payload[1] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(pin_number), 1);
    return request(KINISI_TOGGLE_GPIO_PIN_STATE, payload, 1, nullptr, 0);
}

/** Encode SET_STATUS_LED_STATE, match its reply, and decode the payload. */
bool KinisiController::set_status_led_state(uint8_t state) {
    uint8_t payload[1] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(state), 1);
    return request(KINISI_SET_STATUS_LED_STATE, payload, 1, nullptr, 0);
}

/** Encode TOGGLE_STATUS_LED_STATE, match its reply, and decode the payload. */
bool KinisiController::toggle_status_led_state() {
    uint8_t payload[1] = {0};
    return request(KINISI_TOGGLE_STATUS_LED_STATE, payload, 0, nullptr, 0);
}

/** Encode INITIALIZE_MECANUM_PLATFORM, match its reply, and decode the payload. */
bool KinisiController::initialize_mecanum_platform(bool is_reversed_0, bool is_reversed_1, bool is_reversed_2, bool is_reversed_3, bool is_encoder_reversed_0, bool is_encoder_reversed_1, bool is_encoder_reversed_2, bool is_encoder_reversed_3, double length, double width, double wheels_diameter, double encoder_resolution) {
    uint8_t payload[40] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(is_reversed_0), 1);
    kinisi_codec::writeUnsigned(payload + 1, static_cast<uint64_t>(is_reversed_1), 1);
    kinisi_codec::writeUnsigned(payload + 2, static_cast<uint64_t>(is_reversed_2), 1);
    kinisi_codec::writeUnsigned(payload + 3, static_cast<uint64_t>(is_reversed_3), 1);
    kinisi_codec::writeUnsigned(payload + 4, static_cast<uint64_t>(is_encoder_reversed_0), 1);
    kinisi_codec::writeUnsigned(payload + 5, static_cast<uint64_t>(is_encoder_reversed_1), 1);
    kinisi_codec::writeUnsigned(payload + 6, static_cast<uint64_t>(is_encoder_reversed_2), 1);
    kinisi_codec::writeUnsigned(payload + 7, static_cast<uint64_t>(is_encoder_reversed_3), 1);
    kinisi_codec::writeDouble(payload + 8, length);
    kinisi_codec::writeDouble(payload + 16, width);
    kinisi_codec::writeDouble(payload + 24, wheels_diameter);
    kinisi_codec::writeDouble(payload + 32, encoder_resolution);
    return request(KINISI_INITIALIZE_MECANUM_PLATFORM, payload, 40, nullptr, 0);
}

/** Encode INITIALIZE_OMNI_PLATFORM, match its reply, and decode the payload. */
bool KinisiController::initialize_omni_platform(bool is_reversed_0, bool is_reversed_1, bool is_reversed_2, bool is_encoder_reversed_0, bool is_encoder_reversed_1, bool is_encoder_reversed_2, double wheels_diameter, double robot_radius, double encoder_resolution) {
    uint8_t payload[30] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(is_reversed_0), 1);
    kinisi_codec::writeUnsigned(payload + 1, static_cast<uint64_t>(is_reversed_1), 1);
    kinisi_codec::writeUnsigned(payload + 2, static_cast<uint64_t>(is_reversed_2), 1);
    kinisi_codec::writeUnsigned(payload + 3, static_cast<uint64_t>(is_encoder_reversed_0), 1);
    kinisi_codec::writeUnsigned(payload + 4, static_cast<uint64_t>(is_encoder_reversed_1), 1);
    kinisi_codec::writeUnsigned(payload + 5, static_cast<uint64_t>(is_encoder_reversed_2), 1);
    kinisi_codec::writeDouble(payload + 6, wheels_diameter);
    kinisi_codec::writeDouble(payload + 14, robot_radius);
    kinisi_codec::writeDouble(payload + 22, encoder_resolution);
    return request(KINISI_INITIALIZE_OMNI_PLATFORM, payload, 30, nullptr, 0);
}

/** Encode INITIALIZE_DIFFERENTIAL_PLATFORM, match its reply, and decode the payload. */
bool KinisiController::initialize_differential_platform(bool is_reversed_0, bool is_reversed_1, bool is_encoder_reversed_0, bool is_encoder_reversed_1, double wheel_diameter, double wheel_base, double encoder_resolution) {
    uint8_t payload[28] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(is_reversed_0), 1);
    kinisi_codec::writeUnsigned(payload + 1, static_cast<uint64_t>(is_reversed_1), 1);
    kinisi_codec::writeUnsigned(payload + 2, static_cast<uint64_t>(is_encoder_reversed_0), 1);
    kinisi_codec::writeUnsigned(payload + 3, static_cast<uint64_t>(is_encoder_reversed_1), 1);
    kinisi_codec::writeDouble(payload + 4, wheel_diameter);
    kinisi_codec::writeDouble(payload + 12, wheel_base);
    kinisi_codec::writeDouble(payload + 20, encoder_resolution);
    return request(KINISI_INITIALIZE_DIFFERENTIAL_PLATFORM, payload, 28, nullptr, 0);
}

/** Encode SET_PLATFORM_VELOCITY, match its reply, and decode the payload. */
bool KinisiController::set_platform_velocity(double x, double y, double t) {
    uint8_t payload[24] = {0};
    kinisi_codec::writeDouble(payload + 0, x);
    kinisi_codec::writeDouble(payload + 8, y);
    kinisi_codec::writeDouble(payload + 16, t);
    return request(KINISI_SET_PLATFORM_VELOCITY, payload, 24, nullptr, 0);
}

/** Encode START_PLATFORM_CONTROLLER, match its reply, and decode the payload. */
bool KinisiController::start_platform_controller(double kp, double ki, double kd, double integral_limit) {
    uint8_t payload[32] = {0};
    kinisi_codec::writeDouble(payload + 0, kp);
    kinisi_codec::writeDouble(payload + 8, ki);
    kinisi_codec::writeDouble(payload + 16, kd);
    kinisi_codec::writeDouble(payload + 24, integral_limit);
    return request(KINISI_START_PLATFORM_CONTROLLER, payload, 32, nullptr, 0);
}

/** Encode SET_PLATFORM_TARGET_VELOCITY, match its reply, and decode the payload. */
bool KinisiController::set_platform_target_velocity(double x, double y, double t) {
    uint8_t payload[24] = {0};
    kinisi_codec::writeDouble(payload + 0, x);
    kinisi_codec::writeDouble(payload + 8, y);
    kinisi_codec::writeDouble(payload + 16, t);
    return request(KINISI_SET_PLATFORM_TARGET_VELOCITY, payload, 24, nullptr, 0);
}

/** Encode GET_PLATFORM_CURRENT_VELOCITY, match its reply, and decode the payload. */
platform_velocity KinisiController::get_platform_current_velocity() {
    uint8_t payload[1] = {0};
    platform_velocity result = {};
    uint8_t response[24] = {0};
    if (!request(KINISI_GET_PLATFORM_CURRENT_VELOCITY, payload, 0, response, 24)) return result;
    result.x = kinisi_codec::readDouble(response + 0);
    result.y = kinisi_codec::readDouble(response + 8);
    result.t = kinisi_codec::readDouble(response + 16);
    return result;
}

/** Encode STOP_PLATFORM_CONTROLLER, match its reply, and decode the payload. */
bool KinisiController::stop_platform_controller() {
    uint8_t payload[1] = {0};
    return request(KINISI_STOP_PLATFORM_CONTROLLER, payload, 0, nullptr, 0);
}

/** Encode START_PLATFORM_ODOMETRY, match its reply, and decode the payload. */
bool KinisiController::start_platform_odometry() {
    uint8_t payload[1] = {0};
    return request(KINISI_START_PLATFORM_ODOMETRY, payload, 0, nullptr, 0);
}

/** Encode RESET_PLATFORM_ODOMETRY, match its reply, and decode the payload. */
bool KinisiController::reset_platform_odometry() {
    uint8_t payload[1] = {0};
    return request(KINISI_RESET_PLATFORM_ODOMETRY, payload, 0, nullptr, 0);
}

/** Encode STOP_PLATFORM_ODOMETRY, match its reply, and decode the payload. */
bool KinisiController::stop_platform_odometry() {
    uint8_t payload[1] = {0};
    return request(KINISI_STOP_PLATFORM_ODOMETRY, payload, 0, nullptr, 0);
}

/** Encode GET_PLATFORM_ODOMETRY, match its reply, and decode the payload. */
platform_odometry_sample KinisiController::get_platform_odometry() {
    uint8_t payload[1] = {0};
    platform_odometry_sample result = {};
    uint8_t response[34] = {0};
    if (!request(KINISI_GET_PLATFORM_ODOMETRY, payload, 0, response, 34)) return result;
    result.timestamp_us = static_cast<uint64_t>(kinisi_codec::readUnsigned(response + 0, 8));
    result.clock_mode = static_cast<uint8_t>(kinisi_codec::readUnsigned(response + 8, 1));
    result.clock_quality = static_cast<uint8_t>(kinisi_codec::readUnsigned(response + 9, 1));
    result.x = kinisi_codec::readDouble(response + 10);
    result.y = kinisi_codec::readDouble(response + 18);
    result.t = kinisi_codec::readDouble(response + 26);
    return result;
}

/** Encode BRAKE_PLATFORM, match its reply, and decode the payload. */
bool KinisiController::brake_platform() {
    uint8_t payload[1] = {0};
    return request(KINISI_BRAKE_PLATFORM, payload, 0, nullptr, 0);
}

/** Encode COAST_PLATFORM, match its reply, and decode the payload. */
bool KinisiController::coast_platform() {
    uint8_t payload[1] = {0};
    return request(KINISI_COAST_PLATFORM, payload, 0, nullptr, 0);
}

/** Encode SET_TIME_SYNC_INTERVAL, match its reply, and decode the payload. */
bool KinisiController::set_time_sync_interval(uint32_t interval_ms) {
    uint8_t payload[4] = {0};
    kinisi_codec::writeUnsigned(payload + 0, static_cast<uint64_t>(interval_ms), 4);
    return request(KINISI_SET_TIME_SYNC_INTERVAL, payload, 4, nullptr, 0);
}

/** Encode GET_TIME_STATUS, match its reply, and decode the payload. */
time_status KinisiController::get_time_status() {
    uint8_t payload[1] = {0};
    time_status result = {};
    uint8_t response[14] = {0};
    if (!request(KINISI_GET_TIME_STATUS, payload, 0, response, 14)) return result;
    result.clock_mode = static_cast<uint8_t>(kinisi_codec::readUnsigned(response + 0, 1));
    result.clock_quality = static_cast<uint8_t>(kinisi_codec::readUnsigned(response + 1, 1));
    result.interval_ms = static_cast<uint32_t>(kinisi_codec::readUnsigned(response + 2, 4));
    result.last_sync_age_us = static_cast<uint64_t>(kinisi_codec::readUnsigned(response + 6, 8));
    return result;
}
