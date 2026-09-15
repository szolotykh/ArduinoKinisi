// File: kinisi_protocol.h
// Synchronous API-v2 session state shared by generated command methods.
#ifndef KINISI_PROTOCOL_H
#define KINISI_PROTOCOL_H
#include "i2cutils.h"
#include "kinisi_types.h"

/** Local failures are separate from the controller's shared wire error codes. */
enum class KinisiFailure : uint8_t {
    NONE, NOT_READY, TIMEOUT, TRANSPORT, FRAME_TOO_LARGE,
    MALFORMED_RESPONSE, INCOMPATIBLE_PROTOCOL, CONTROLLER
};
/** Result of the most recent operation; zero-valued getter results may be errors. */
struct KinisiError {
    KinisiFailure failure;
    KinisiErrorCode code;
    uint8_t command;
    uint16_t message_id;
};

class KinisiProtocol {
public:
    /** Create an inactive session. Request IDs persist across begin() calls. */
    explicit KinisiProtocol(uint8_t address);
    /** Await uptime INIT/READY and enable heartbeat; pass zero to opt out of monitoring. */
    bool begin(uint32_t timeout_ms = 1000, uint32_t heartbeat_timeout_ms = 500);
    /** Call frequently from loop(): drain due telemetry and send PING only after idle. */
    bool poll();
    /** Copy the latest streamed encoder measurement; false until a sample is available. */
    bool getSubscribedEncoderOdometry(uint8_t index, encoder_odometry_sample& sample) const;
    /** Copy the latest streamed platform measurement; false until a sample is available. */
    bool getSubscribedPlatformOdometry(platform_odometry_sample& sample) const;
    /** Whether the last handshake completed and transport remains usable. */
    bool ready() const { return ready_; }
    /** Board identity from the most recent successful INIT. */
    const init_response& boardInfo() const { return identity_; }
    /** Failure of the most recent operation; inspect after every value-returning getter. */
    const KinisiError& lastError() const { return error_; }
protected:
    /** Execute exactly one request; never retry an operation that might have executed. */
    bool request(uint8_t command, const uint8_t* payload, uint8_t length,
                 uint8_t* response, uint8_t response_length);
private:
    uint8_t address_;
    uint16_t next_id_;
    uint32_t timeout_ms_;
    bool ready_;
    init_response identity_;
    KinisiError error_;
    uint32_t heartbeat_ms_, last_sent_ms_, last_poll_ms_;
    uint32_t subscription_ms_[5];
    uint8_t sample_mask_;
    encoder_odometry_sample encoder_samples_[4];
    platform_odometry_sample platform_sample_;
    /** Validate and cache one unsolicited ID-zero telemetry frame. */
    bool telemetry(const uint8_t* frame, uint8_t size);
    /** Set structured failure; fatal transport/protocol failures invalidate the session. */
    bool fail(KinisiFailure failure, uint8_t command, uint16_t id,
              KinisiErrorCode code = KinisiErrorCode::NONE);
    /** Write a frame only after validating request and response buffer limits. */
    bool send(uint8_t command, uint16_t id, const uint8_t* payload,
              uint8_t length, uint8_t response_length);
    /** Match command and message ID, ignore old IDs, and decode shared ERROR frames. */
    bool receive(uint8_t expected_command, uint8_t failed_command, uint16_t id,
                 uint8_t* payload, uint8_t length, uint32_t started);
    /** Return the next nonzero ID; wrap 65535 to 1. */
    uint16_t allocateId();
};
#endif
