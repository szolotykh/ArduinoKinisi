// Public board profiles and sensor commands through the actual I2C session.
#include <assert.h>
#include <stdio.h>
#include "kinisi.h"

TwoWire Wire;
MockSerial Serial;
uint32_t fake_millis = 0;
static uint8_t revision = 4, patch = 0, protocol = 5;

static std::vector<uint8_t> frame(uint8_t command, uint16_t id, std::vector<uint8_t> payload = {}) {
    std::vector<uint8_t> result = {uint8_t(payload.size() + 3), command, uint8_t(id), uint8_t(id >> 8)};
    result.insert(result.end(), payload.begin(), payload.end());
    return result;
}

static std::vector<uint8_t> measurements(bool imu) {
    // Golden IEEE binary64 values: -1.25, 0, 9.5, 0.125, -0.25, 0.5; temperature -0.0625.
    const uint64_t values[] = {UINT64_C(0xbff4000000000000), 0, UINT64_C(0x4023000000000000),
        UINT64_C(0x3fc0000000000000), UINT64_C(0xbfd0000000000000), UINT64_C(0x3fe0000000000000)};
    std::vector<uint8_t> result;
    for (unsigned i = 0; i < (imu ? 6U : 1U); ++i) {
        uint64_t bits = imu ? values[i] : UINT64_C(0xbfb0000000000000);
        for (unsigned j = 0; j < 8; ++j) result.push_back(uint8_t(bits >> (j * 8)));
    }
    return result;
}

static void respond(const std::vector<uint8_t>& request) {
    const uint8_t command = request[1];
    const uint16_t id = request[2] | uint16_t(request[3]) << 8;
    if (command == KINISI_INIT) {
        Wire.responses.push_back(frame(command, id, {1, 0, revision, patch, 2, protocol, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
        Wire.responses.push_back(frame(KINISI_READY, id, {0}));
    } else if (command == KINISI_GET_TEMPERATURE || command == KINISI_GET_IMU) {
        assert(request.size() == 5 && request[4] == 0);
        Wire.responses.push_back(frame(command, id, measurements(command == KINISI_GET_IMU)));
    } else Wire.responses.push_back(frame(command, id));
}

static void reset(uint8_t board = 4, uint8_t minor = 5, uint8_t board_patch = 0) {
    Wire = TwoWire(); fake_millis = 0;
    revision = board; protocol = minor; patch = board_patch;
    Wire.responder = respond;
}

static void profiles() {
    const uint8_t encoder_counts[] = {1, 4, 4, 4}, gpio_counts[] = {4, 4, 10, 8};
    KinisiBoardProfile profile;
    for (uint8_t i = 1; i <= 4; ++i) {
        assert(kinisiLookupBoardProfile(1, 0, i, 0, profile));
        assert(profile.model == 1 && profile.major == 0 && profile.minor == i && profile.patch == 0);
        assert(profile.resources.motors == 4 && profile.resources.encoders == encoder_counts[i - 1]);
        assert(profile.resources.gpio == gpio_counts[i - 1] && profile.resources.status_leds == 1);
        for (uint8_t motor = 0; motor < 4; ++motor) {
            assert(profile.motor_features[motor].id == motor);
            assert(profile.motor_features[motor].current_sense == (i == 4));
            assert(profile.motor_features[motor].current_limit == (i == 4));
        }
        assert(profile.hasSensor(KinisiSensor::TEMPERATURE) == (i == 4));
        assert(profile.hasSensor(KinisiSensor::IMU) == (i == 4));
        assert(!profile.hasSensor(KinisiSensor::IMU, 1));
        assert(!profile.hasSensor(static_cast<KinisiSensor>(255)));
    }
    assert(profile.imu[0].accelerometer && profile.imu[0].gyroscope);
    assert(!kinisiLookupBoardProfile(1, 0, 4, 1, profile));
    assert(profile.model == 0 && profile.resources.motors == 0);
    assert(!kinisiLookupBoardProfile(2, 0, 4, 0, profile));
    assert(!kinisiLookupBoardProfile(1, 0, 99, 0, profile));
}

static void sensor_reads_and_capacity() {
    reset(); KinisiController c;
    assert(c.boardProfile() == nullptr && !c.supportsSensor(KinisiSensor::IMU));
    assert(c.get_temperature(0) == 0 && c.lastError().failure == KinisiFailure::NOT_READY);
    assert(Wire.transactions == 0);
    assert(c.begin(1000, 0));
    assert(c.boardProfile() && c.boardProfile()->minor == 4);
    assert(c.supportsSensor(KinisiSensor::TEMPERATURE));
    assert(c.get_temperature(0) == -0.0625 && c.lastError().failure == KinisiFailure::NONE);
    size_t before = Wire.transactions;
    imu_sample sample = c.get_imu(0);
#if KINISI_WIRE_BUFFER_SIZE >= 52
    assert(c.supportsSensor(KinisiSensor::IMU));
    assert(c.lastError().failure == KinisiFailure::NONE && Wire.transactions == before + 1);
    assert(sample.acceleration_x == -1.25 && sample.acceleration_y == 0 && sample.acceleration_z == 9.5);
    assert(sample.angular_velocity_x == .125 && sample.angular_velocity_y == -.25 && sample.angular_velocity_z == .5);
#else
    assert(!c.supportsSensor(KinisiSensor::IMU));
    assert(sample.acceleration_z == 0 && c.lastError().failure == KinisiFailure::FRAME_TOO_LARGE);
    assert(Wire.transactions == before && c.ready());
#endif
    before = Wire.transactions;
    c.get_temperature(255);
    assert(c.lastError().failure == KinisiFailure::SENSOR_NOT_AVAILABLE && Wire.transactions == before);
    assert(c.ready() && !c.supportsSensor(KinisiSensor::TEMPERATURE, 255));
}

static void gating_and_unknown_boards() {
    for (uint8_t minor = 1; minor < 5; ++minor) {
        reset(4, minor); KinisiController c; assert(c.begin(1000, 0));
        size_t before = Wire.transactions;
        c.get_imu(0);
        assert(c.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND);
        if (minor < 4) {
            c.get_temperature(0);
            assert(c.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND);
            assert(!c.supportsSensor(KinisiSensor::TEMPERATURE));
        }
        assert(Wire.transactions == before && c.ready());
        assert(c.toggle_status_led_state());
    }
    for (uint8_t board = 1; board <= 3; ++board) {
        reset(board); KinisiController c; assert(c.begin(1000, 0));
        assert(c.boardProfile() && !c.supportsSensor(KinisiSensor::TEMPERATURE));
        size_t before = Wire.transactions;
        c.get_temperature(0);
        assert(c.lastError().failure == KinisiFailure::SENSOR_NOT_AVAILABLE);
        assert(c.lastError().code == KinisiErrorCode::SENSOR_NOT_AVAILABLE);
        c.get_imu(0);
        assert(c.lastError().failure == KinisiFailure::SENSOR_NOT_AVAILABLE && Wire.transactions == before);
    }
    for (uint8_t board : {uint8_t(4), uint8_t(99)}) {
        reset(board, 5, 1); KinisiController c; assert(c.begin(1000, 0));
        assert(c.boardProfile() == nullptr && !c.supportsSensor(KinisiSensor::TEMPERATURE));
        assert(c.get_temperature(0) == -0.0625 && c.lastError().failure == KinisiFailure::NONE);
    }
}

static void errors_and_lifecycle() {
    reset(); KinisiController c; assert(c.begin(1000, 0));
    for (uint8_t code : {uint8_t(6), uint8_t(10), uint8_t(16)}) {
        Wire.responder = [code](const std::vector<uint8_t>& r) {
            Wire.responses.push_back(frame(KINISI_ERROR, r[2] | uint16_t(r[3]) << 8, {r[1], code}));
        };
        c.get_temperature(0);
        assert(c.lastError().failure == KinisiFailure::CONTROLLER && uint8_t(c.lastError().code) == code);
        assert(c.lastError().command == KINISI_GET_TEMPERATURE);
        assert(c.lastError().message_id == (Wire.sent[2] | uint16_t(Wire.sent[3]) << 8));
        assert(c.ready() && c.boardProfile());
    }
    Wire.responder = respond;
    assert(c.get_temperature(0) == -0.0625 && c.lastError().failure == KinisiFailure::NONE);
    reset(3); assert(c.begin(1000, 0));
    assert(c.boardProfile()->minor == 3 && !c.supportsSensor(KinisiSensor::TEMPERATURE));
    reset(); assert(c.begin(5, 0)); Wire.responder = nullptr;
    c.get_temperature(0);
    assert(c.lastError().failure == KinisiFailure::TIMEOUT && !c.ready());
    assert(c.boardProfile() == nullptr && c.boardInfo().board_model == 0);
    reset(); assert(c.begin(5, 0));
    Wire.responder = [](const std::vector<uint8_t>& r) { respond(r); Wire.responses.pop_back(); };
    assert(!c.begin(5, 0) && c.boardProfile() == nullptr);
}

static void invalid_samples() {
    for (bool imu : {false, true}) {
        if (imu && KINISI_WIRE_BUFFER_SIZE < 52) continue;
        for (unsigned field = 0; field < (imu ? 6U : 1U); ++field) {
            for (uint64_t bits : {UINT64_C(0x7ff8000000000000), UINT64_C(0x7ff0000000000000), UINT64_C(0xfff0000000000000)}) {
                reset(); KinisiController c; assert(c.begin(1000, 0));
                Wire.responder = [imu, field, bits](const std::vector<uint8_t>& r) {
                    auto payload = measurements(imu);
                    for (unsigned i = 0; i < 8; ++i) payload[field * 8 + i] = uint8_t(bits >> (i * 8));
                    Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3]) << 8, payload));
                };
                if (imu) assert(c.get_imu(0).acceleration_z == 0);
                else assert(c.get_temperature(0) == 0);
                assert(c.lastError().failure == KinisiFailure::MALFORMED_RESPONSE && !c.ready());
                assert(c.boardProfile() == nullptr);
            }
        }
        reset(); KinisiController c; assert(c.begin(1000, 0));
        Wire.responder = [imu](const std::vector<uint8_t>& r) {
            auto payload = measurements(imu); payload.pop_back();
            Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3]) << 8, payload));
        };
        if (imu) c.get_imu(0); else c.get_temperature(0);
        assert(c.lastError().failure == KinisiFailure::MALFORMED_RESPONSE && !c.ready());
    }
}

int main() {
    profiles(); sensor_reads_and_capacity(); gating_and_unknown_boards(); errors_and_lifecycle(); invalid_samples();
    printf("PASS board profiles, sensors and Wire capacity %u\n", unsigned(KINISI_WIRE_BUFFER_SIZE));
}
