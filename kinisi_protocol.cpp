// File: kinisi_protocol.cpp
// INIT/READY, ID correlation, ACK and ERROR handling for the blocking Arduino SDK.
#include "kinisi_protocol.h"
#include "kinisi_codec.h"
#include <string.h>

/** Initialize local state without starting any bus traffic. */
KinisiProtocol::KinisiProtocol(uint8_t address)
    : address_(address), next_id_(1), timeout_ms_(1000), ready_(false), identity_(), error_() {}

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
        ready_ = false;
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
    return true;
}

/** Consume complete Wire transactions until the matching response or timeout arrives. */
bool KinisiProtocol::receive(uint8_t expected_command, uint8_t failed_command, uint16_t id,
                             uint8_t* payload, uint8_t length, uint32_t started) {
    uint8_t read_size = length + 4;
    if (read_size < 6) read_size = 6;
    uint8_t frame[64];
    while (uint32_t(millis() - started) < timeout_ms_) {
        int count = kinisiWireReceive(address_, frame, read_size);
        if (count < 0) return fail(KinisiFailure::TRANSPORT, failed_command, id);
        if (count == 0) { delay(1); continue; }
        if (count < 4 || frame[0] < 3 || uint16_t(frame[0]) + 1 > uint16_t(count))
            return fail(KinisiFailure::MALFORMED_RESPONSE, failed_command, id);
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
bool KinisiProtocol::begin(uint32_t timeout_ms) {
    ready_ = false;
    identity_ = {};
    error_ = {};
    timeout_ms_ = timeout_ms ? timeout_ms : 1000;
    kinisiWireBegin();
    uint16_t id = allocateId();
    const uint8_t init[] = {3, 2, 0, 0, 2, 0, 0, 0};
    uint8_t response[15];
    uint32_t started = millis();
    if (!send(KINISI_INIT, id, init, sizeof(init), sizeof(response)) ||
        !receive(KINISI_INIT, KINISI_INIT, id, response, sizeof(response), started)) return false;
    if (response[4] != 2)
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
    return true;
}

/** Issue a correlated request and wait for its ACK or typed response. */
bool KinisiProtocol::request(uint8_t command, const uint8_t* payload, uint8_t length,
                             uint8_t* response, uint8_t response_length) {
    error_ = {};
    if (!ready_) return fail(KinisiFailure::NOT_READY, command, 0);
    uint16_t id = allocateId();
    uint32_t started = millis();
    return send(command, id, payload, length, response_length) &&
           receive(command, command, id, response, response_length, started);
}
