// File: kinisi_protocol.cpp
// INIT/READY, ID correlation, ACK and ERROR handling for the blocking Arduino SDK.
#include "kinisi_protocol.h"
#include "kinisi_codec.h"
#include <string.h>

/** Initialize local state without starting any bus traffic. */
KinisiProtocol::KinisiProtocol(uint8_t address)
    : address_(address), next_id_(1), timeout_ms_(1000), ready_(false), identity_(), error_(),
      heartbeat_ms_(0), last_sent_ms_(0), last_poll_ms_(0), subscription_ms_{}, sample_mask_(0),
      encoder_samples_{}, platform_sample_() {}

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
        heartbeat_ms_ = 0; sample_mask_ = 0;
        memset(subscription_ms_, 0, sizeof(subscription_ms_));
    }
    return false;
}

/** Check all limits before sending; a rejected oversized command has no side effects. */
bool KinisiProtocol::send(uint8_t command, uint16_t id, const uint8_t* payload,
                          uint8_t length, uint8_t response_length) {
    // Every generated v2 request and response fits a 64-byte scratch buffer.
    uint16_t read_size = response_length + 4;
    if (read_size < 6) read_size = 6; // ERROR contains failed command + code.
    if (uint16_t(length) + 4 > 64 || read_size > 64 ||
        uint16_t(length) + 4 > KINISI_WIRE_BUFFER_SIZE || read_size > KINISI_WIRE_BUFFER_SIZE)
        return fail(KinisiFailure::FRAME_TOO_LARGE, command, id);
    uint8_t frame[64];
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
        if (frame[1] == KINISI_ENCODER_ODOMETRY_EVENT || frame[1] == KINISI_PLATFORM_ODOMETRY_EVENT) {
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
    error_ = {};
    heartbeat_ms_ = 0; sample_mask_ = 0;
    memset(subscription_ms_, 0, sizeof(subscription_ms_));
    timeout_ms_ = timeout_ms ? timeout_ms : 1000;
    kinisiWireBegin();
    uint16_t id = allocateId();
    const uint8_t init[] = {3, 2, 1, 0, 2, 1, 0, 2};
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
    return true;
}

/** Issue a correlated request and wait for its ACK or typed response. */
bool KinisiProtocol::request(uint8_t command, const uint8_t* payload, uint8_t length,
                             uint8_t* response, uint8_t response_length) {
    error_ = {};
    if (!ready_) return fail(KinisiFailure::NOT_READY, command, 0);
    if (command == KINISI_SUBSCRIBE_ODOMETRY && length == 5 && payload[0] == 4 && KINISI_WIRE_BUFFER_SIZE < 38)
        return fail(KinisiFailure::FRAME_TOO_LARGE, command, 0);
    uint16_t id = allocateId();
    uint32_t started = millis();
    if (!send(command, id, payload, length, response_length) ||
        !receive(command, command, id, response, response_length, started)) return false;
    if (command == KINISI_SET_HEARTBEAT_CONFIG) {
        heartbeat_ms_ = payload[0] ? uint32_t(kinisi_codec::readUnsigned(payload + 1, 4)) : 0;
        if (!heartbeat_ms_) { memset(subscription_ms_, 0, sizeof(subscription_ms_)); sample_mask_ = 0; }
    } else if (command == KINISI_SUBSCRIBE_ODOMETRY && payload[0] < 5) {
        subscription_ms_[payload[0]] = uint32_t(kinisi_codec::readUnsigned(payload + 1, 4));
        last_poll_ms_ = millis();
    } else if (command == KINISI_UNSUBSCRIBE_ODOMETRY && payload[0] < 5) {
        subscription_ms_[payload[0]] = 0;
        sample_mask_ &= ~(1U << payload[0]);
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
