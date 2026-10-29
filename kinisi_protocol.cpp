// File: kinisi_protocol.cpp
// INIT/READY, ID correlation, ACK and ERROR handling for the blocking Arduino SDK.
#include "kinisi_protocol.h"
#include "kinisi_codec.h"
#include <string.h>
#include <math.h>

/** Initialize local state without starting any bus traffic. */
KinisiProtocol::KinisiProtocol(uint8_t address)
    : address_(address), next_id_(1), timeout_ms_(1000), ready_(false), identity_(), profile_(), profile_known_(false), error_(),
      heartbeat_ms_(0), last_sent_ms_(0), last_poll_ms_(0), subscription_ms_{}, sample_mask_(0),
      encoder_samples_{}, platform_sample_(), subscriptions_{} {}

void KinisiProtocol::clearSubscriptions() {
    memset(subscriptions_, 0, sizeof(subscriptions_));
}

int KinisiProtocol::findSubscription(KinisiTopic topic, uint8_t index) const {
    for (uint8_t i = 0; i < KINISI_MAX_SUBSCRIPTIONS; ++i)
        if (subscriptions_[i].interval_ms && subscriptions_[i].sample.topic == topic && subscriptions_[i].sample.index == index)
            return i;
    return -1;
}

/** Advance the request counter; zero is reserved by API v2. */
uint16_t KinisiProtocol::allocateId() {
    uint16_t id = next_id_++;
    if (next_id_ == 0) next_id_ = 1;
    return id;
}

/** Preserve controller errors while invalidating sessions with uncertain transport state. */
bool KinisiProtocol::fail(KinisiFailure failure, uint8_t command, uint16_t id, KinisiErrorCode code) {
    error_ = {failure, code, command, id};
    if (failure == KinisiFailure::TIMEOUT || failure == KinisiFailure::TRANSPORT ||
        failure == KinisiFailure::MALFORMED_RESPONSE || failure == KinisiFailure::INCOMPATIBLE_PROTOCOL ||
        code == KinisiErrorCode::INIT_REQUIRED || code == KinisiErrorCode::CLOCK_NOT_READY)
    {
        ready_ = false;
        identity_ = {}; profile_ = {}; profile_known_ = false;
        heartbeat_ms_ = 0; sample_mask_ = 0;
        memset(subscription_ms_, 0, sizeof(subscription_ms_));
        clearSubscriptions();
    }
    return false;
}

/** Check all limits before sending; a rejected oversized command has no side effects. */
bool KinisiProtocol::send(uint8_t command, uint16_t id, const uint8_t* payload,
                          uint8_t length, uint8_t response_length) {
    // Platform position PID uses a 100-byte request. Never split a frame;
    // both this scratch buffer and the board's real Wire buffer must fit it.
    uint16_t read_size = response_length + 4;
    if (read_size < 6) read_size = 6; // ERROR contains failed command + code.
    if (uint16_t(length) + 4 > 128 || read_size > 64 ||
        uint16_t(length) + 4 > KINISI_WIRE_BUFFER_SIZE || read_size > KINISI_WIRE_BUFFER_SIZE)
        return fail(KinisiFailure::FRAME_TOO_LARGE, command, id);
    uint8_t frame[128];
    frame[0] = length + 3;
    frame[1] = command;
    kinisi_codec::writeUnsigned(frame + 2, id, 2);
    if (length) memcpy(frame + 4, payload, length);
    if (!kinisiWireSend(address_, frame, length + 4))
        return fail(KinisiFailure::TRANSPORT, command, id);
    last_sent_ms_ = millis();
    return true;
}

/** Consume complete Wire transactions until the matching response or timeout arrives. */
bool KinisiProtocol::receive(uint8_t expected_command, uint8_t failed_command, uint16_t id,
                             uint8_t* payload, uint8_t length, uint32_t started) {
    // A telemetry event may precede any pending reply on the same I2C stream.
    const uint8_t read_size = KINISI_WIRE_BUFFER_SIZE < 64 ? KINISI_WIRE_BUFFER_SIZE : 64;
    uint8_t frame[64];
    while (uint32_t(millis() - started) < timeout_ms_) {
        int count = kinisiWireReceive(address_, frame, read_size);
        if (count < 0) return fail(KinisiFailure::TRANSPORT, failed_command, id);
        if (count == 0) { delay(1); continue; }
        if (count < 4 || frame[0] < 3 || uint16_t(frame[0]) + 1 > uint16_t(count))
            return fail(KinisiFailure::MALFORMED_RESPONSE, failed_command, id);
        if (frame[1] == KINISI_ENCODER_ODOMETRY_EVENT || frame[1] == KINISI_PLATFORM_ODOMETRY_EVENT || frame[1] == KINISI_TELEMETRY_EVENT) {
            if (!telemetry(frame, frame[0] + 1)) return fail(KinisiFailure::MALFORMED_RESPONSE, failed_command, id);
            continue;
        }
        if (kinisi_codec::readUnsigned(frame + 2, 2) != id) { delay(1); continue; }
        if (frame[1] == KINISI_ERROR) {
            if (frame[0] != 5 || frame[4] != failed_command || frame[5] == 0)
                return fail(KinisiFailure::MALFORMED_RESPONSE, failed_command, id);
            return fail(KinisiFailure::CONTROLLER, failed_command, id, static_cast<KinisiErrorCode>(frame[5]));
        }
        if (frame[1] != expected_command || frame[0] != length + 3)
            return fail(KinisiFailure::MALFORMED_RESPONSE, failed_command, id);
        if (length) memcpy(payload, frame + 4, length);
        return true; // Ignore padding beyond the declared frame, never interpret it as another frame.
    }
    return fail(KinisiFailure::TIMEOUT, failed_command, id);
}

/** Advertise uptime-only operation and require READY before allowing user requests. */
bool KinisiProtocol::begin(uint32_t timeout_ms, uint32_t heartbeat_timeout_ms) {
    ready_ = false;
    identity_ = {};
    profile_ = {}; profile_known_ = false;
    error_ = {};
    heartbeat_ms_ = 0; sample_mask_ = 0;
    memset(subscription_ms_, 0, sizeof(subscription_ms_));
    clearSubscriptions();
    timeout_ms_ = timeout_ms ? timeout_ms : 1000;
    kinisiWireBegin();
    uint16_t id = allocateId();
    const uint8_t init[] = {3, 2, 5, 0, 2, 1, 0, 2};
    uint8_t response[15];
    uint32_t started = millis();
    if (!send(KINISI_INIT, id, init, sizeof(init), sizeof(response)) ||
        !receive(KINISI_INIT, KINISI_INIT, id, response, sizeof(response), started)) return false;
    if (response[4] != 2 || response[5] < 1)
        return fail(KinisiFailure::INCOMPATIBLE_PROTOCOL, KINISI_INIT, id);
    identity_.board_model = response[0];
    identity_.board_major = response[1];
    identity_.board_minor = response[2];
    identity_.board_patch = response[3];
    identity_.protocol_major = response[4];
    identity_.protocol_minor = response[5];
    identity_.protocol_patch = response[6];
    identity_.firmware_build_high = uint32_t(kinisi_codec::readUnsigned(response + 7, 4));
    identity_.firmware_build_low = uint32_t(kinisi_codec::readUnsigned(response + 11, 4));
    uint8_t mode;
    if (!receive(KINISI_READY, KINISI_INIT, id, &mode, 1, started)) return false;
    if (mode != 0) return fail(KinisiFailure::MALFORMED_RESPONSE, KINISI_INIT, id);
    ready_ = true;
    if (heartbeat_timeout_ms) {
        uint8_t config[5] = {1};
        kinisi_codec::writeUnsigned(config + 1, heartbeat_timeout_ms, 4);
        if (!request(KINISI_SET_HEARTBEAT_CONFIG, config, 5, nullptr, 0)) { ready_ = false; return false; }
    }
    profile_known_ = kinisiLookupBoardProfile(identity_.board_model, identity_.board_major,
        identity_.board_minor, identity_.board_patch, profile_);
    return true;
}

bool KinisiProtocol::supportsSensor(KinisiSensor kind, uint8_t index) const {
    if (!ready_ || !profile_known_ || !profile_.hasSensor(kind, index) || identity_.protocol_major != 2)
        return false;
    switch (kind) {
        case KinisiSensor::TEMPERATURE: return identity_.protocol_minor >= 4 && KINISI_WIRE_BUFFER_SIZE >= 12;
        case KinisiSensor::IMU: return identity_.protocol_minor >= 5 && KINISI_WIRE_BUFFER_SIZE >= 52;
        default: return false;
    }
}

const KinisiPwmFrequencyOptions* KinisiProtocol::getPwmFrequencyOptions() const {
    return profile_known_ && profile_.pwm_frequency.frequency_count ? &profile_.pwm_frequency : nullptr;
}
bool KinisiProtocol::supportsPwmFrequency() const {
    return ready_ && identity_.protocol_major == 2 && identity_.protocol_minor >= 13 &&
        KINISI_WIRE_BUFFER_SIZE >= 8 && getPwmFrequencyOptions();
}

const KinisiCurrentLimitOptions* KinisiProtocol::getMotorCurrentLimitOptions(uint8_t index) const {
    if (!profile_known_) return nullptr;
    for (uint8_t i = 0; i < profile_.resources.motors; ++i) {
        const KinisiMotorFeatures& motor = profile_.motor_features[i];
        if (motor.id == index && motor.current_limit && motor.current_limit_options.threshold_count)
            return &motor.current_limit_options;
    }
    return nullptr;
}

bool KinisiProtocol::supportsMotorCurrentLimit(uint8_t index) const {
    return ready_ && identity_.protocol_major == 2 && identity_.protocol_minor >= 11 &&
        KINISI_WIRE_BUFFER_SIZE >= 8 && getMotorCurrentLimitOptions(index);
}

bool KinisiProtocol::supportsMotorCurrent(uint8_t index) const {
    if (!ready_ || !profile_known_ || identity_.protocol_major != 2 || identity_.protocol_minor < 7 ||
            KINISI_WIRE_BUFFER_SIZE < 12) return false;
    for (uint8_t i = 0; i < profile_.resources.motors; ++i)
        if (profile_.motor_features[i].id == index) return profile_.motor_features[i].current_sense;
    return false;
}

bool KinisiProtocol::supportsMotorFaults(uint8_t index) const {
    if (!ready_ || !profile_known_ || identity_.protocol_major != 2 || identity_.protocol_minor < 8 ||
            KINISI_WIRE_BUFFER_SIZE < 14) return false;
    for (uint8_t i = 0; i < profile_.resources.motors; ++i)
        if (profile_.motor_features[i].id == index) return profile_.motor_features[i].fault_reporting;
    return false;
}

bool KinisiProtocol::supportsTopic(KinisiTopic topic, uint8_t index) const {
    uint8_t size; uint32_t minimum;
    if (!ready_ || !profile_known_ || identity_.protocol_major != 2 || identity_.protocol_minor < 6 ||
            !kinisiTopicInfo(topic, index, size, minimum) || size + 6 > KINISI_WIRE_BUFFER_SIZE || size + 6 > 64)
        return false;
    switch (topic) {
        case KinisiTopic::ENCODER_ODOMETRY: return index < profile_.resources.encoders;
        case KinisiTopic::PLATFORM_ODOMETRY: return true;
        case KinisiTopic::TEMPERATURE: return profile_.hasSensor(KinisiSensor::TEMPERATURE, index);
        case KinisiTopic::IMU: return profile_.hasSensor(KinisiSensor::IMU, index);
        case KinisiTopic::MOTOR_FAULTS: return supportsMotorFaults(index);
        case KinisiTopic::MOTOR_CURRENT: return supportsMotorCurrent(index);
        default: return false;
    }
}

bool KinisiProtocol::getSubscriptionSample(KinisiTopic topic, uint8_t index, KinisiTelemetrySample& sample) const {
    int slot = findSubscription(topic, index);
    if (!ready_ || slot < 0 || !subscriptions_[slot].has_sample) return false;
    sample = subscriptions_[slot].sample;
    return true;
}

/** Reject invalid velocity gains and integral limits before transmission. */
bool KinisiProtocol::validateVelocityTuning(uint8_t command, double kp, double ki, double kd, double limit) {
    if (!isfinite(kp) || kp < 0 || !isfinite(ki) || ki < 0 ||
        !isfinite(kd) || kd < 0 || !isfinite(limit) || limit < 0 || limit > 100)
        return fail(KinisiFailure::INVALID_ARGUMENT, command, 0, KinisiErrorCode::INVALID_ARGUMENT);
    return true;
}

bool KinisiProtocol::request(uint8_t command, const uint8_t* payload, uint8_t length,
                             uint8_t* response, uint8_t response_length) {
    error_ = {};
    if (!ready_) return fail(KinisiFailure::NOT_READY, command, 0);
    uint8_t required_minor = 0;
    switch (command) {
        case KINISI_INITIALIZE_MOTOR_POSITION_CONTROLLER:
        case KINISI_RESET_MOTOR_POSITION:
        case KINISI_SET_MOTOR_POSITION:
        case KINISI_GET_MOTOR_POSITION:
        case KINISI_INITIALIZE_PLATFORM_POSITION_CONTROLLER:
        case KINISI_RESET_PLATFORM_POSITION:
        case KINISI_SET_PLATFORM_POSITION: required_minor = 2; break;
        case KINISI_INITIALIZE_MOTOR_POSITION_PID_CONTROLLER:
        case KINISI_INITIALIZE_PLATFORM_POSITION_PID_CONTROLLER: required_minor = 3; break;
        case KINISI_GET_TEMPERATURE: required_minor = 4; break;
        case KINISI_GET_IMU: required_minor = 5; break;
        case KINISI_GET_MOTOR_CURRENT_LIMIT:
        case KINISI_SET_MOTOR_CURRENT_LIMIT: required_minor = 11; break;
        case KINISI_GET_PWM_FREQUENCY:
        case KINISI_SET_PWM_FREQUENCY: required_minor = 13; break;
        case KINISI_CLEAR_MOTOR_FAULTS: required_minor = 10; break;
        case KINISI_GET_MOTOR_FAULTS: required_minor = 8; break;
        case KINISI_GET_MOTOR_CURRENT: required_minor = 7; break;
        case KINISI_SUBSCRIBE_CONFIGURED: required_minor = 12; break;
        case KINISI_SUBSCRIBE_WITH_MODE: required_minor = 9; break;
        case KINISI_SUBSCRIBE:
        case KINISI_UNSUBSCRIBE: required_minor = 6; break;
        default: break;
    }
    if (identity_.protocol_minor < required_minor)
        return fail(KinisiFailure::UNSUPPORTED_COMMAND, command, 0);
    int subscription_slot = -1;
    if (command == KINISI_SUBSCRIBE || command == KINISI_SUBSCRIBE_WITH_MODE || command == KINISI_SUBSCRIBE_CONFIGURED || command == KINISI_UNSUBSCRIBE) {
        const KinisiTopic topic = static_cast<KinisiTopic>(payload[0]);
        if (topic == KinisiTopic::MOTOR_FAULTS && identity_.protocol_minor < 8)
            return fail(KinisiFailure::UNSUPPORTED_COMMAND, command, 0);
        if (topic == KinisiTopic::MOTOR_CURRENT && identity_.protocol_minor < 7)
            return fail(KinisiFailure::UNSUPPORTED_COMMAND, command, 0);
        uint8_t size; uint32_t minimum;
        if (!kinisiTopicInfo(topic, payload[1], size, minimum))
            return fail(KinisiFailure::INVALID_ARGUMENT, command, 0);
        subscription_slot = findSubscription(topic, payload[1]);
        if (command == KINISI_SUBSCRIBE || command == KINISI_SUBSCRIBE_WITH_MODE || command == KINISI_SUBSCRIBE_CONFIGURED) {
            uint32_t interval = uint32_t(kinisi_codec::readUnsigned(payload + 2, 4));
            if (command == KINISI_SUBSCRIBE_CONFIGURED) {
                if (length != 12) return fail(KinisiFailure::INVALID_ARGUMENT, command, 0);
                uint32_t silence = uint32_t(kinisi_codec::readUnsigned(payload + 8, 4));
                bool event = payload[6] == uint8_t(KinisiSubscriptionTrigger::ON_EVENT);
                if (payload[6] > 1 || payload[7] > 1 || interval > 3600000 || silence > 3600000 ||
                    (event ? (interval != 0 || !kinisiTopicEventSupported(topic)) : interval < minimum) ||
                    (silence && (payload[7] != uint8_t(KinisiSubscriptionDelivery::ON_CHANGE) || silence < minimum)))
                    return fail(KinisiFailure::INVALID_ARGUMENT, command, 0);
            } else if ((command == KINISI_SUBSCRIBE_WITH_MODE && (length != 7 || payload[6] > uint8_t(KinisiSubscriptionMode::ON_CHANGE))) ||
                interval < minimum || interval > 3600000)
                return fail(KinisiFailure::INVALID_ARGUMENT, command, 0);
            if (size + 6 > KINISI_WIRE_BUFFER_SIZE || size + 6 > 64)
                return fail(KinisiFailure::FRAME_TOO_LARGE, command, 0);
            if (profile_known_ && !supportsTopic(topic, payload[1]))
                return fail(KinisiFailure::SENSOR_NOT_AVAILABLE, command, 0);
            if (subscription_slot < 0) {
                for (uint8_t i = 0; i < KINISI_MAX_SUBSCRIPTIONS; ++i)
                    if (!subscriptions_[i].interval_ms) { subscription_slot = i; break; }
            }
            if (subscription_slot < 0) return fail(KinisiFailure::SUBSCRIPTION_LIMIT, command, 0);
        }
    }
    if (command == KINISI_GET_PWM_FREQUENCY || command == KINISI_SET_PWM_FREQUENCY) {
        const KinisiPwmFrequencyOptions* options = getPwmFrequencyOptions();
        if (profile_known_ && !options) return fail(KinisiFailure::SENSOR_NOT_AVAILABLE, command, 0, KinisiErrorCode::SENSOR_NOT_AVAILABLE);
        if (command == KINISI_SET_PWM_FREQUENCY) {
            uint32_t frequency = uint32_t(kinisi_codec::readUnsigned(payload, 4));
            bool allowed = options == nullptr;
            if (options) for (uint8_t i = 0; i < options->frequency_count; ++i) allowed |= options->supported_hz[i] == frequency;
            if (!allowed || !kinisiPwmFrequencyValid(frequency)) return fail(KinisiFailure::INVALID_ARGUMENT, command, 0, KinisiErrorCode::INVALID_ARGUMENT);
        }
    }
    const bool current_limit = command == KINISI_GET_MOTOR_CURRENT_LIMIT || command == KINISI_SET_MOTOR_CURRENT_LIMIT;
    if (current_limit) {
        if (payload[0] > 3) return fail(KinisiFailure::INVALID_ARGUMENT, command, 0, KinisiErrorCode::INVALID_ARGUMENT);
        const KinisiCurrentLimitOptions* options = getMotorCurrentLimitOptions(payload[0]);
        if (profile_known_ && !options)
            return fail(KinisiFailure::SENSOR_NOT_AVAILABLE, command, 0, KinisiErrorCode::SENSOR_NOT_AVAILABLE);
        if (command == KINISI_SET_MOTOR_CURRENT_LIMIT) {
            uint16_t threshold = uint16_t(kinisi_codec::readUnsigned(payload + 2, 2));
            bool allowed = options == nullptr;
            if (options) for (uint8_t i = 0; i < options->threshold_count; ++i)
                allowed |= threshold == options->supported_thresholds_ma[i];
            if (!threshold || !allowed || (options && !payload[1] && !options->can_disable))
                return fail(KinisiFailure::INVALID_ARGUMENT, command, 0, KinisiErrorCode::INVALID_ARGUMENT);
        }
    }
    const bool sensor_read = command == KINISI_GET_TEMPERATURE || command == KINISI_GET_IMU;
    const bool fault_read = command == KINISI_GET_MOTOR_FAULTS || command == KINISI_CLEAR_MOTOR_FAULTS;
    if (fault_read && payload[0] > 3)
        return fail(KinisiFailure::INVALID_ARGUMENT, command, 0, KinisiErrorCode::INVALID_ARGUMENT);
    if (fault_read && profile_known_ && !supportsMotorFaults(payload[0]))
        return fail(KinisiFailure::SENSOR_NOT_AVAILABLE, command, 0, KinisiErrorCode::SENSOR_NOT_AVAILABLE);
    const bool current_read = command == KINISI_GET_MOTOR_CURRENT;
    if (current_read && payload[0] > 3)
        return fail(KinisiFailure::INVALID_ARGUMENT, command, 0, KinisiErrorCode::INVALID_ARGUMENT);
    if (current_read && profile_known_ && !supportsMotorCurrent(payload[0]))
        return fail(KinisiFailure::SENSOR_NOT_AVAILABLE, command, 0, KinisiErrorCode::SENSOR_NOT_AVAILABLE);
    if (sensor_read && profile_known_ && !profile_.hasSensor(
            command == KINISI_GET_TEMPERATURE ? KinisiSensor::TEMPERATURE : KinisiSensor::IMU, payload[0]))
        return fail(KinisiFailure::SENSOR_NOT_AVAILABLE, command, 0, KinisiErrorCode::SENSOR_NOT_AVAILABLE);
    if (command == KINISI_SUBSCRIBE_ODOMETRY && length == 5 && payload[0] == 4 && KINISI_WIRE_BUFFER_SIZE < 38)
        return fail(KinisiFailure::FRAME_TOO_LARGE, command, 0);
    uint16_t id = allocateId();
    uint32_t started = millis();
    if (!send(command, id, payload, length, response_length) ||
        !receive(command, command, id, response, response_length, started)) return false;
    if (current_limit && (response[0] > 1 || !kinisi_codec::readUnsigned(response + 1, 2)))
        return fail(KinisiFailure::MALFORMED_RESPONSE, command, id);
    if (fault_read) {
        for (uint8_t i = 0; i < response_length; ++i)
            if (response[i] > 1) return fail(KinisiFailure::MALFORMED_RESPONSE, command, id);
    }
    if (current_read && kinisi_codec::readDouble(response) < 0)
        return fail(KinisiFailure::MALFORMED_RESPONSE, command, id);
    if (sensor_read || current_read) {
        for (uint8_t offset = 0; offset < response_length; offset += 8)
            if (!isfinite(kinisi_codec::readDouble(response + offset)))
                return fail(KinisiFailure::MALFORMED_RESPONSE, command, id);
    }
    if (command == KINISI_SET_HEARTBEAT_CONFIG) {
        heartbeat_ms_ = payload[0] ? uint32_t(kinisi_codec::readUnsigned(payload + 1, 4)) : 0;
        if (!heartbeat_ms_) { memset(subscription_ms_, 0, sizeof(subscription_ms_)); sample_mask_ = 0; clearSubscriptions(); }
    } else if (command == KINISI_SUBSCRIBE || command == KINISI_SUBSCRIBE_WITH_MODE || command == KINISI_SUBSCRIBE_CONFIGURED || command == KINISI_UNSUBSCRIBE) {
        KinisiTopic topic = static_cast<KinisiTopic>(payload[0]);
        if (subscription_slot >= 0) {
            SubscriptionSlot& slot = subscriptions_[subscription_slot];
            slot = {};
            if (command == KINISI_SUBSCRIBE || command == KINISI_SUBSCRIBE_WITH_MODE || command == KINISI_SUBSCRIBE_CONFIGURED) {
                slot.interval_ms = uint32_t(kinisi_codec::readUnsigned(payload + 2, 4));
                if (command == KINISI_SUBSCRIBE_CONFIGURED) {
                    uint32_t silence = uint32_t(kinisi_codec::readUnsigned(payload + 8, 4));
                    if (!slot.interval_ms) slot.interval_ms = 10; // I2C master services event notifications.
                    if (silence && silence < slot.interval_ms) slot.interval_ms = silence;
                }
                slot.sample.topic = topic;
                slot.sample.index = payload[1];
                last_poll_ms_ = millis();
            }
        }
        int source = topic == KinisiTopic::PLATFORM_ODOMETRY ? 4 : topic == KinisiTopic::ENCODER_ODOMETRY ? payload[1] : -1;
        if (source >= 0) { subscription_ms_[source] = 0; sample_mask_ &= ~(1U << source); }
    } else if (command == KINISI_SUBSCRIBE_ODOMETRY && payload[0] < 5) {
        subscription_ms_[payload[0]] = uint32_t(kinisi_codec::readUnsigned(payload + 1, 4));
        last_poll_ms_ = millis();
    } else if (command == KINISI_UNSUBSCRIBE_ODOMETRY && payload[0] < 5) {
        subscription_ms_[payload[0]] = 0;
        sample_mask_ &= ~(1U << payload[0]);
    }
    if ((command == KINISI_SUBSCRIBE_ODOMETRY || command == KINISI_UNSUBSCRIBE_ODOMETRY) && payload[0] < 5) {
        KinisiTopic topic = payload[0] == 4 ? KinisiTopic::PLATFORM_ODOMETRY : KinisiTopic::ENCODER_ODOMETRY;
        int slot = findSubscription(topic, payload[0] == 4 ? 0 : payload[0]);
        if (slot >= 0) subscriptions_[slot] = {};
    }
    return true;
}

/** Clock out one due event and its ACK; otherwise refresh only an idle connection. */
bool KinisiProtocol::poll() {
    if (!ready_) return fail(KinisiFailure::NOT_READY, KINISI_PING, 0);
    uint32_t shortest = UINT32_MAX;
    uint8_t count = 0;
    for (uint8_t i = 0; i < 5; ++i)
        if (subscription_ms_[i]) {
            ++count;
            if (subscription_ms_[i] < shortest) shortest = subscription_ms_[i];
        }
    for (uint8_t i = 0; i < KINISI_MAX_SUBSCRIPTIONS; ++i)
        if (subscriptions_[i].interval_ms) {
            ++count;
            if (subscriptions_[i].interval_ms < shortest) shortest = subscriptions_[i].interval_ms;
        }
    // Service each subscribed source; firmware owns the independent delivery deadlines.
    uint32_t service_ms = count ? shortest / count : shortest;
    if (!service_ms) service_ms = 1;
    if (shortest != UINT32_MAX && uint32_t(millis() - last_poll_ms_) >= service_ms) {
        last_poll_ms_ = millis();
        return request(KINISI_POLL_TELEMETRY, nullptr, 0, nullptr, 0);
    }
    if (heartbeat_ms_ && uint32_t(millis() - last_sent_ms_) >= heartbeat_ms_ / 5)
        return request(KINISI_PING, nullptr, 0, nullptr, 0);
    return true;
}

/** Decode packed wire values explicitly, including binary64 on AVR's 32-bit double. */
bool KinisiProtocol::telemetry(const uint8_t* frame, uint8_t size) {
    if (frame[1] == KINISI_TELEMETRY_EVENT) {
        if (kinisi_codec::readUnsigned(frame + 2, 2) || size < 6) return false;
        KinisiTopic topic = static_cast<KinisiTopic>(frame[4]);
        uint8_t payload_size; uint32_t minimum;
        // Unknown future topics are ignored; malformed known topics are errors.
        if (!kinisiTopicInfo(topic, 0, payload_size, minimum)) return true;
        KinisiTelemetrySample sample = {};
        if (!kinisiDecodeTelemetry(topic, frame[5], frame + 6, size - 6, sample)) return false;
        int slot = findSubscription(topic, frame[5]);
        if (ready_ && slot >= 0) {
            subscriptions_[slot].sample = sample;
            subscriptions_[slot].has_sample = true;
        }
        return true;
    }
    bool encoder = frame[1] == KINISI_ENCODER_ODOMETRY_EVENT;
    if (kinisi_codec::readUnsigned(frame + 2, 2) || size != (encoder ? 23 : 38)) return false;
    uint8_t source = encoder ? frame[4] : 4;
    if (source > 4 || (encoder && source == 4)) return false;
    const uint8_t* p = frame + (encoder ? 5 : 4);
    if (p[8] > 1 || p[9] < 1 || p[9] > 2) return false;
    if (encoder) {
        encoder_samples_[source] = {kinisi_codec::readUnsigned(p, 8), p[8], p[9], kinisi_codec::readDouble(p + 10)};
    } else {
        platform_sample_ = {kinisi_codec::readUnsigned(p, 8), p[8], p[9], kinisi_codec::readDouble(p + 10),
            kinisi_codec::readDouble(p + 18), kinisi_codec::readDouble(p + 26)};
    }
    if (ready_) sample_mask_ |= 1U << source;
    return true;
}

/** Copy an available cached sample without performing I2C traffic. */
bool KinisiProtocol::getSubscribedEncoderOdometry(uint8_t index, encoder_odometry_sample& sample) const {
    if (index > 3 || !(sample_mask_ & (1U << index))) return false;
    sample = encoder_samples_[index]; return true;
}

/** Copy the most recent platform event without triggering another calculation. */
bool KinisiProtocol::getSubscribedPlatformOdometry(platform_odometry_sample& sample) const {
    if (!(sample_mask_ & 16)) return false;
    sample = platform_sample_; return true;
}
