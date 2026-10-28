// Generic topics through the real Wire session, including constrained buffers.
#include "kinisi.h"
#include "kinisi_codec.h"
#include <assert.h>
#include <stdio.h>

TwoWire Wire;
MockSerial Serial;
uint32_t fake_millis = 0;
static uint8_t board = 4, minor = 7, event_topic = 3, event_index = 0;
static double value = 12.5;

static std::vector<uint8_t> frame(uint8_t command, uint16_t id, std::vector<uint8_t> data = {}) {
    std::vector<uint8_t> result = {uint8_t(data.size() + 3), command, uint8_t(id), uint8_t(id >> 8)};
    result.insert(result.end(), data.begin(), data.end());
    return result;
}

static std::vector<uint8_t> event() {
    uint8_t size = event_topic == 2 ? 34 : event_topic == 4 ? 58 : event_topic == 6 ? 20 : 18;
    std::vector<uint8_t> payload(size + 2);
    payload[0] = event_topic; payload[1] = event_index;
    kinisi_codec::writeUnsigned(payload.data() + 2, UINT64_C(36028797018963975), 8);
    payload[10] = 0; payload[11] = 1;
    if (event_topic == 6) { payload[13] = 1; payload[18] = value != 0; }
    else for (uint8_t offset = 12; offset < payload.size(); offset += 8)
        kinisi_codec::writeDouble(payload.data() + offset, value);
    return frame(KINISI_TELEMETRY_EVENT, 0, payload);
}

static void respond(const std::vector<uint8_t>& r) {
    uint8_t command = r[1]; uint16_t id = r[2] | uint16_t(r[3]) << 8;
    if (command == KINISI_INIT) {
        Wire.responses.push_back(frame(command, id, {1, 0, board, 0, 2, minor, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
        Wire.responses.push_back(frame(KINISI_READY, id, {0}));
    } else {
        if (command == KINISI_POLL_TELEMETRY) Wire.responses.push_back(event());
        Wire.responses.push_back(frame(command, id));
    }
}

static void reset(uint8_t revision = 4, uint8_t protocol = 8) {
    Wire = TwoWire(); fake_millis = 0; board = revision; minor = protocol; value = 12.5;
    Wire.responder = respond;
}

static void topics_and_buffers() {
    reset(); KinisiController c; assert(c.begin());
    for (uint8_t topic = 1; topic <= 6; ++topic) {
        KinisiTopic t = static_cast<KinisiTopic>(topic);
        uint8_t size; uint32_t minimum;
        assert(kinisiTopicInfo(t, 0, size, minimum));
        size_t before = Wire.transactions;
        if (size + 6 > KINISI_WIRE_BUFFER_SIZE) {
            assert(!c.supportsTopic(t));
            assert(!c.subscribe(t, 0, 500));
            assert(c.lastError().failure == KinisiFailure::FRAME_TOO_LARGE && Wire.transactions == before);
            continue;
        }
        assert(c.supportsTopic(t));
        assert(c.subscribe(t, 0, 500));
        assert(Wire.sent == frame(KINISI_SUBSCRIBE, Wire.sent[2] | uint16_t(Wire.sent[3]) << 8,
                                 {topic, 0, 0xf4, 0x01, 0, 0}));
        event_topic = topic; event_index = 0;
        fake_millis += 500; assert(c.poll());
        KinisiTelemetrySample sample;
        assert(c.getSubscriptionSample(t, 0, sample));
        assert(sample.topic == t && sample.index == 0);
        switch (t) {
            case KinisiTopic::ENCODER_ODOMETRY: assert(sample.data.encoder_odometry.angle == value); break;
            case KinisiTopic::PLATFORM_ODOMETRY: assert(sample.data.platform_odometry.t == value); break;
            case KinisiTopic::TEMPERATURE: assert(sample.data.temperature.celsius == value); break;
            case KinisiTopic::IMU: assert(sample.data.imu.angular_velocity_z == value); break;
            case KinisiTopic::MOTOR_FAULTS: assert(sample.data.motor_faults.status_valid && sample.data.motor_faults.overcurrent); break;
            case KinisiTopic::MOTOR_CURRENT: assert(sample.data.motor_current.amps == value); break;
        }
        value = 9.0; fake_millis += 500; assert(c.poll());
        assert(c.getSubscriptionSample(t, 0, sample));
        assert(c.subscribe(t, 0, 1000)); // Replace clears the previous interval's sample.
        assert(!c.getSubscriptionSample(t, 0, sample));
        assert(c.unsubscribe(t, 0));
        assert(!c.getSubscriptionSample(t, 0, sample));
        assert(c.unsubscribe(t, 0));
    }
}

static void validation_and_lifecycle() {
    reset(); KinisiController c; assert(c.begin());
    size_t before = Wire.transactions;
    assert(!c.subscribe(KinisiTopic::TEMPERATURE, 0, 249));
    assert(!c.subscribe(KinisiTopic::PLATFORM_ODOMETRY, 1, 100));
    assert(!c.subscribe(static_cast<KinisiTopic>(99), 0, 100));
    assert(!c.subscribe(KinisiTopic::TEMPERATURE, 1, 250));
    assert(Wire.transactions == before);
    assert(c.subscribe(KinisiTopic::TEMPERATURE, 0, 250));
    event_topic = 3; event_index = 0; fake_millis += 250; assert(c.poll());
    KinisiTelemetrySample sample;
    assert(c.getSubscriptionSample(KinisiTopic::TEMPERATURE, 0, sample));
    assert(c.set_heartbeat_config(false, 500));
    assert(!c.getSubscriptionSample(KinisiTopic::TEMPERATURE, 0, sample));
    assert(c.begin());
    assert(c.subscribe(KinisiTopic::ENCODER_ODOMETRY, 0, 100));
    assert(c.subscribe_odometry(0, 100));
    assert(!c.getSubscriptionSample(KinisiTopic::ENCODER_ODOMETRY, 0, sample));
    reset(3); assert(c.begin());
    before = Wire.transactions;
    assert(!c.subscribe(KinisiTopic::TEMPERATURE, 0, 250));
    assert(Wire.transactions == before && c.ready());
    reset(4, 5); assert(c.begin());
    before = Wire.transactions;
    assert(!c.supportsTopic(KinisiTopic::TEMPERATURE));
    assert(!c.subscribe(KinisiTopic::TEMPERATURE, 0, 250));
    assert(c.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND && Wire.transactions == before);
    reset(99); assert(c.begin()); // Unknown profiles defer presence checks to firmware.
    assert(!c.supportsTopic(KinisiTopic::TEMPERATURE));
    for (uint8_t i = 0; i < KINISI_MAX_SUBSCRIPTIONS; ++i) assert(c.subscribe(KinisiTopic::TEMPERATURE, i, 250));
    before = Wire.transactions;
    assert(!c.subscribe(KinisiTopic::TEMPERATURE, KINISI_MAX_SUBSCRIPTIONS, 250));
    assert(c.lastError().failure == KinisiFailure::SUBSCRIPTION_LIMIT && Wire.transactions == before);
    assert(c.unsubscribe(KinisiTopic::TEMPERATURE, 0));
    assert(c.subscribe(KinisiTopic::TEMPERATURE, KINISI_MAX_SUBSCRIPTIONS, 250));
}

static void malformed_and_future_events() {
    reset(); KinisiController c; assert(c.begin());
    assert(c.subscribe(KinisiTopic::TEMPERATURE, 0, 250));
    Wire.responder = [](const std::vector<uint8_t>& r) {
        Wire.responses.push_back(frame(KINISI_TELEMETRY_EVENT, 0, {99, 0}));
        Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3]) << 8));
    };
    assert(c.ping() && c.ready());
    for (uint8_t bad = 0; bad < 4; ++bad) {
        reset(); assert(c.begin());
        assert(c.subscribe(KinisiTopic::TEMPERATURE, 0, 250));
        event_topic = 3; event_index = 0;
        Wire.responder = [bad](const std::vector<uint8_t>& r) {
            auto bytes = event();
            if (bad == 0) bytes[2] = 1; // Nonzero unsolicited message ID.
            if (bad == 1) { bytes.pop_back(); --bytes[0]; }
            if (bad == 2) bytes[15] = 0; // Invalid clock quality.
            if (bad == 3) kinisi_codec::writeUnsigned(bytes.data() + 16, UINT64_C(0x7ff8000000000000), 8);
            Wire.responses.push_back(bytes);
            Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3]) << 8));
        };
        assert(!c.ping() && !c.ready());
        assert(c.lastError().failure == KinisiFailure::MALFORMED_RESPONSE);
    }
}

static void four_current_streams() {
    reset(); KinisiController c; assert(c.begin());
    for (uint8_t index = 0; index < 4; ++index) {
        assert(c.supportsTopic(KinisiTopic::MOTOR_CURRENT, index));
        assert(c.subscribe(KinisiTopic::MOTOR_CURRENT, index, 20));
    }
    event_topic = 5;
    for (uint8_t index = 0; index < 4; ++index) {
        event_index = index; value = index + .125;
        fake_millis += 20; assert(c.poll());
    }
    for (uint8_t index = 0; index < 4; ++index) {
        KinisiTelemetrySample sample;
        assert(c.getSubscriptionSample(KinisiTopic::MOTOR_CURRENT, index, sample));
        assert(sample.data.motor_current.amps == index + .125);
        assert(c.unsubscribe(KinisiTopic::MOTOR_CURRENT, index));
        assert(!c.getSubscriptionSample(KinisiTopic::MOTOR_CURRENT, index, sample));
    }
    auto bytes = event();
    kinisi_codec::writeDouble(bytes.data() + 16, -1);
    KinisiTelemetrySample sample;
    assert(!kinisiDecodeTelemetry(KinisiTopic::MOTOR_CURRENT, 3, bytes.data() + 6, 18, sample));
}


static void on_change_mode() {
    reset(4, 9); KinisiController c; assert(c.begin());
    auto topic = KinisiTopic::MOTOR_FAULTS;
    size_t before = Wire.transactions;
    assert(!c.subscribe(topic, 0, 100, static_cast<KinisiSubscriptionMode>(2)));
    assert(c.lastError().failure == KinisiFailure::INVALID_ARGUMENT && Wire.transactions == before);
    assert(c.subscribe(topic, 0, 100, KinisiSubscriptionMode::ON_CHANGE));
    assert(Wire.sent == frame(KINISI_SUBSCRIBE_WITH_MODE, Wire.sent[2] | uint16_t(Wire.sent[3]) << 8,
                             {6, 0, 100, 0, 0, 0, 1}));
    event_topic = 6; event_index = 0; fake_millis += 100;
    assert(c.poll()); KinisiTelemetrySample sample;
    assert(c.getSubscriptionSample(topic, 0, sample) && sample.data.motor_faults.overcurrent);
    // No new events is normal, and cached state remains available.
    Wire.responder = [](const std::vector<uint8_t>& r) {
        Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3]) << 8));
    };
    for (unsigned i = 0; i < 20; ++i) { fake_millis += 100; assert(c.poll() && c.ready()); }
    assert(c.getSubscriptionSample(topic, 0, sample));
    assert(c.subscribe(topic, 0, 100)); assert(Wire.sent[1] == KINISI_SUBSCRIBE);
    assert(!c.getSubscriptionSample(topic, 0, sample));
    assert(c.unsubscribe(topic, 0));
    reset(4, 8); assert(c.begin()); before = Wire.transactions;
    assert(!c.subscribe(topic, 0, 100, KinisiSubscriptionMode::ON_CHANGE));
    assert(c.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND && Wire.transactions == before);
    assert(c.subscribe(topic, 0, 100));
    reset(99, 8); assert(c.begin()); before = Wire.transactions;
    assert(!c.subscribe(topic, 0, 100, KinisiSubscriptionMode::ON_CHANGE));
    assert(Wire.transactions == before);
}

static void configured_settings() {
    reset(4, 12); KinisiController c; assert(c.begin());
    assert(c.subscribe(KinisiTopic::MOTOR_FAULTS, 0, 0, KinisiSubscriptionTrigger::ON_EVENT, KinisiSubscriptionDelivery::ON_CHANGE, 5000));
    assert(Wire.sent == frame(KINISI_SUBSCRIBE_CONFIGURED, Wire.sent[2] | uint16_t(Wire.sent[3]) << 8,
        {6, 0, 0, 0, 0, 0, 1, 1, 0x88, 0x13, 0, 0}));
    event_topic = 6; event_index = 0; fake_millis += 10; assert(c.poll());
    KinisiTelemetrySample sample; assert(c.getSubscriptionSample(KinisiTopic::MOTOR_FAULTS, 0, sample));
    size_t count = Wire.transactions;
    assert(!c.subscribe(KinisiTopic::MOTOR_FAULTS, 0, 100, KinisiSubscriptionTrigger::ON_EVENT, KinisiSubscriptionDelivery::ON_CHANGE, 5000));
    assert(!c.subscribe(KinisiTopic::TEMPERATURE, 0, 0, KinisiSubscriptionTrigger::ON_EVENT, KinisiSubscriptionDelivery::ALL));
    assert(!c.subscribe(KinisiTopic::MOTOR_FAULTS, 0, 0, KinisiSubscriptionTrigger::ON_EVENT, KinisiSubscriptionDelivery::ALL, 5000));
    assert(Wire.transactions == count);
    assert(c.unsubscribe(KinisiTopic::MOTOR_FAULTS, 0));
    reset(4, 11); KinisiController old; assert(old.begin()); count = Wire.transactions;
    assert(!old.subscribe(KinisiTopic::MOTOR_FAULTS, 0, 0, KinisiSubscriptionTrigger::ON_EVENT, KinisiSubscriptionDelivery::ALL));
    assert(old.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND && Wire.transactions == count);
}

int main() {
    configured_settings();
    on_change_mode();
    topics_and_buffers(); validation_and_lifecycle(); malformed_and_future_events();
    four_current_streams();
    printf("PASS generic subscriptions, lifecycle and Wire capacity %d\n", KINISI_WIRE_BUFFER_SIZE);
}
