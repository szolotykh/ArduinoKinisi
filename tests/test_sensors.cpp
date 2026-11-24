// Public board profiles and sensor commands through the actual I2C session.
#include <assert.h>
#include <stdio.h>
#include "kinisi.h"
#include "kinisi_codec.h"
#include "legacy_profile.h"

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
    assert(!legacy_profile.capabilities.fusion_experiment && !legacy_profile_v215.capabilities.fusion_experiment);
    assert(legacy_profile.resources.motors == 4 && legacy_profile.pwm_frequency.default_hz == 100000UL);
    assert(legacy_profile.motor_features[3].id == 3 && legacy_profile.motor_features[3].current_limit);
    assert(legacy_profile.hasSensor(KinisiSensor::IMU) && legacy_profile.imu[0].gyroscope);
    assert(!legacy_profile.power.driver_sleep);
    assert(!legacy_profile.power.board_idle);
    assert(legacy_profile_v215.motor_features[3].id == 3 && legacy_profile_v215.imu[0].gyroscope);
    assert(legacy_profile_v215.power.driver_sleep && !legacy_profile_v215.power.board_idle);
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
        assert(profile.power.driver_sleep == (i >= 3));
        assert(profile.power.board_idle == (i >= 3));
        assert(!profile.hasSensor(KinisiSensor::IMU, 1));
        assert(!profile.hasSensor(static_cast<KinisiSensor>(255)));
    }
    assert(profile.imu[0].accelerometer && profile.imu[0].gyroscope);
    assert(kinisiLookupBoardProfile(1, 0, 3, 1, profile));
    assert(profile.patch == 1 && profile.resources.motors == 4 && profile.resources.encoders == 4);
    assert(profile.power.driver_sleep && profile.power.board_idle);
    assert(!kinisiLookupBoardProfile(1, 0, 3, 2, profile));
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

static void current_readings() {
    reset(4, 7); KinisiController c; assert(c.begin(1000, 0));
    Wire.responder = [](const std::vector<uint8_t>& r) {
        assert(r[1] == KINISI_GET_MOTOR_CURRENT && r.size() == 5 && r[4] < 4);
        Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3]) << 8, {0, 0, 0, 0, 0, 0, 0xf4, 0x3f}));
    };
    for (uint8_t i = 0; i < 4; ++i) {
        assert(c.supportsMotorCurrent(i) && c.get_motor_current(i) == 1.25);
        assert(c.lastError().failure == KinisiFailure::NONE);
    }
    size_t before = Wire.transactions;
    assert(!c.supportsMotorCurrent(4));
    c.get_motor_current(4);
    assert(c.lastError().failure == KinisiFailure::INVALID_ARGUMENT && Wire.transactions == before);
    for (uint8_t board : {1, 2, 3, 4, 99}) {
        reset(board, board >= 4 ? 6 : 7); assert(c.begin(1000, 0));
        before = Wire.transactions;
        assert(!c.supportsMotorCurrent());
        c.get_motor_current(0); assert(c.lastError().failure != KinisiFailure::NONE);
        assert(!c.subscribe(KinisiTopic::MOTOR_CURRENT, 0, 100));
        assert(Wire.transactions == before);
    }
    for (uint64_t bits : {UINT64_C(0xbff0000000000000), UINT64_C(0x7ff8000000000000), UINT64_C(0x7ff0000000000000)}) {
        reset(4, 7); assert(c.begin(1000, 0));
        Wire.responder = [bits](const std::vector<uint8_t>& r) {
            std::vector<uint8_t> bytes;
            for (unsigned i = 0; i < 8; ++i) bytes.push_back(uint8_t(bits >> (i * 8)));
            Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3]) << 8, bytes));
        };
        assert(c.get_motor_current(0) == 0 && c.lastError().failure == KinisiFailure::MALFORMED_RESPONSE);
    }
}


static void fault_readings() {
    reset(4, 8); KinisiController c; assert(c.begin(1000, 0));
    Wire.responder = [](const std::vector<uint8_t>& r) {
        assert(r[1] == KINISI_GET_MOTOR_FAULTS && r.size() == 5 && r[4] < 4);
        Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3]) << 8,
            {0, 1, 0, 0, 0, 0, uint8_t(r[4] == 2), 0, 0, 0}));
    };
    for (uint8_t i = 0; i < 4; ++i) {
        assert(c.supportsMotorFaults(i));
        motor_fault_status status = c.get_motor_faults(i);
        assert(status.status_valid && status.overcurrent == (i == 2));
        assert(c.lastError().failure == KinisiFailure::NONE);
    }
    size_t before = Wire.transactions;
    c.get_motor_faults(4);
    assert(c.lastError().failure == KinisiFailure::INVALID_ARGUMENT && Wire.transactions == before);
    for (uint8_t revision : {1, 2, 3, 4, 99}) {
        reset(revision, revision >= 4 ? 7 : 8); assert(c.begin(1000, 0));
        before = Wire.transactions;
        assert(!c.supportsMotorFaults()); c.get_motor_faults(0);
        assert(c.lastError().failure != KinisiFailure::NONE);
        assert(!c.subscribe(KinisiTopic::MOTOR_FAULTS, 0, 100));
        assert(Wire.transactions == before);
    }
    reset(4, 8); assert(c.begin(1000, 0));
    Wire.responder = [](const std::vector<uint8_t>& r) {
        Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3]) << 8, std::vector<uint8_t>(10, 0)));
    };
    assert(!c.get_motor_faults(0).status_valid && c.lastError().failure == KinisiFailure::NONE);
    Wire.responder = [](const std::vector<uint8_t>& r) {
        Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3]) << 8, {0, 2, 0, 0, 0, 0, 0, 0, 0, 0}));
    };
    c.get_motor_faults(0); assert(c.lastError().failure == KinisiFailure::MALFORMED_RESPONSE);
}


static void clear_faults() {
    reset(4, 10); KinisiController c; assert(c.begin(1000, 0));
    Wire.responder = [](const std::vector<uint8_t>& r) {
        assert(r[1] == KINISI_CLEAR_MOTOR_FAULTS && r.size() == 5 && r[4] < 4);
        Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3]) << 8, {0, 1, 1, 0, 0, 0, 1, 0, 0, 0}));
    };
    for (uint8_t i = 0; i < 4; ++i) {
        motor_fault_status status = c.clear_motor_faults(i);
        assert(c.lastError().failure == KinisiFailure::NONE && status.status_valid && status.overcurrent);
    }
    size_t before = Wire.transactions; c.clear_motor_faults(4);
    assert(c.lastError().failure == KinisiFailure::INVALID_ARGUMENT && Wire.transactions == before);
    for (uint8_t revision : {1, 2, 3, 4, 99}) {
        reset(revision, revision >= 4 ? 9 : 10); assert(c.begin(1000, 0)); before = Wire.transactions;
        c.clear_motor_faults(0); assert(c.lastError().failure != KinisiFailure::NONE && Wire.transactions == before);
    }
    reset(4, 10); assert(c.begin(1000, 0));
    Wire.responder = [](const std::vector<uint8_t>& r) {
        Wire.responses.push_back(frame(KINISI_ERROR, r[2] | uint16_t(r[3]) << 8,
            {r[1], uint8_t(KinisiErrorCode::INTERNAL_ERROR)}));
    };
    c.clear_motor_faults(0); assert(c.lastError().code == KinisiErrorCode::INTERNAL_ERROR && c.ready());
    Wire.responder = [](const std::vector<uint8_t>& r) {
        Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3]) << 8, {0, 2, 0, 0, 0, 0, 0, 0, 0, 0}));
    };
    c.clear_motor_faults(0); assert(c.lastError().failure == KinisiFailure::MALFORMED_RESPONSE);
}

static void current_limits() {
    reset(4, 11); KinisiController c; assert(c.begin(1000, 0));
    for (uint8_t index = 0; index < 4; ++index) {
        assert(c.supportsMotorCurrentLimit(index));
        const KinisiCurrentLimitOptions* options = c.getMotorCurrentLimitOptions(index);
        assert(options && options->threshold_count == 4 && options->can_disable);
        const uint16_t expected[] = {4000, 5400, 6500, 7000};
        for (uint8_t i = 0; i < 4; ++i) assert(options->supported_thresholds_ma[i] == expected[i]);
        Wire.responder = [index](const std::vector<uint8_t>& r) {
            assert(r[4] == index);
            if (r[1] == KINISI_SET_MOTOR_CURRENT_LIMIT) {
                assert(r.size() == 8 && r[5] == 0 && r[6] == 0x18 && r[7] == 0x15);
            } else assert(r[1] == KINISI_GET_MOTOR_CURRENT_LIMIT && r.size() == 5);
            Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3]) << 8, {0, 0x18, 0x15}));
        };
        motor_current_limit setting = c.set_motor_current_limit(index, false, 5400);
        assert(c.lastError().failure == KinisiFailure::NONE && !setting.enabled && setting.threshold_ma == 5400);
        setting = c.get_motor_current_limit(index);
        assert(c.lastError().failure == KinisiFailure::NONE && setting.threshold_ma == 5400);
    }
    size_t before = Wire.transactions;
    c.set_motor_current_limit(0, true, 5000);
    assert(c.lastError().failure == KinisiFailure::INVALID_ARGUMENT && Wire.transactions == before);
    c.get_motor_current_limit(4);
    assert(c.lastError().failure == KinisiFailure::INVALID_ARGUMENT && Wire.transactions == before);
    Wire.responder = [](const std::vector<uint8_t>& r) {
        Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3]) << 8, {2, 0x18, 0x15}));
    };
    c.get_motor_current_limit(0);
    assert(c.lastError().failure == KinisiFailure::MALFORMED_RESPONSE);
    Wire.responder = [](const std::vector<uint8_t>&) {};
    assert(!c.begin(10, 0));
    assert(!c.supportsMotorCurrentLimit() && c.getMotorCurrentLimitOptions() == nullptr);
    for (uint8_t board = 1; board <= 4; ++board) {
        reset(board, board == 4 ? 10 : 11); assert(c.begin(1000, 0));
        assert(!c.supportsMotorCurrentLimit());
        before = Wire.transactions;
        c.set_motor_current_limit(0, true, 5400);
        assert(Wire.transactions == before && c.lastError().failure != KinisiFailure::NONE);
    }
}

static void pwm_frequencies() {
    KinisiController c;
    for (uint8_t board = 1; board <= 4; ++board) {
        reset(board, 13); assert(c.begin(1000, 0));
        assert(c.supportsPwmFrequency());
        const KinisiPwmFrequencyOptions* options = c.getPwmFrequencyOptions();
        assert(options && options->default_hz == 100000UL && options->frequency_count == 5);
        const uint32_t expected[] = {10000UL, 20000UL, 25000UL, 50000UL, 100000UL};
        for (uint8_t i=0; i<5; ++i) assert(options->supported_hz[i] == expected[i]);
        Wire.responder = [](const std::vector<uint8_t>& r) {
            if (r[1] == KINISI_SET_PWM_FREQUENCY) assert(r.size() == 8 && r[4] == 0xa8 && r[5] == 0x61 && r[6] == 0 && r[7] == 0);
            else assert(r[1] == KINISI_GET_PWM_FREQUENCY && r.size() == 4);
            Wire.responses.push_back(frame(r[1], r[2] | uint16_t(r[3]) << 8, {0xa8, 0x61, 0, 0}));
        };
        assert(c.set_pwm_frequency(KinisiPwmFrequency::KHZ_25) == 25000UL);
        assert(c.lastError().failure == KinisiFailure::NONE);
        assert(c.get_pwm_frequency() == 25000UL);
        size_t before = Wire.transactions;
        c.set_pwm_frequency(static_cast<KinisiPwmFrequency>(12345));
        assert(c.lastError().failure == KinisiFailure::INVALID_ARGUMENT && Wire.transactions == before);
    }
    reset(4, 12); assert(c.begin(1000, 0));
    assert(!c.supportsPwmFrequency());
    size_t before = Wire.transactions;
    c.set_pwm_frequency(KinisiPwmFrequency::KHZ_25); c.get_pwm_frequency();
    assert(c.lastError().failure != KinisiFailure::NONE && Wire.transactions == before);
    reset(99, 13); assert(c.begin(1000, 0));
    assert(!c.supportsPwmFrequency() && !c.getPwmFrequencyOptions());
    before = Wire.transactions;
    c.set_pwm_frequency(static_cast<KinisiPwmFrequency>(12345));
    assert(c.lastError().failure == KinisiFailure::INVALID_ARGUMENT && Wire.transactions == before);
}

static void configuration_save() {
    KinisiController c;
    for (uint8_t board = 1; board <= 5; ++board) {
        reset(board, 14);
        assert(c.begin(1000, 0));
        assert(c.supportsConfigurationSave() == (board == 4));
        const auto before = Wire.transactions;
        assert(c.save_configuration() == (board == 4));
        if (board != 4) {
            assert(c.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND);
            assert(Wire.transactions == before);
        }
    }
    reset(4, 13);
    assert(c.begin(1000, 0));
    assert(!c.supportsConfigurationSave() && !c.save_configuration());
    reset(4, 14);
    assert(c.begin(1000, 0));
    Wire.responder = [](const std::vector<uint8_t>& r) {
        const uint16_t id = r[2] | uint16_t(r[3]) << 8;
        if (r[1] == KINISI_SAVE_CONFIGURATION) {
            assert(r.size() == 4);
            Wire.responses.push_back(frame(KINISI_ERROR, id, {r[1], uint8_t(KinisiErrorCode::INTERNAL_ERROR)}));
        } else {
            assert(r[1] == KINISI_GET_CONFIGURATION_FIELD && r.size() == 6 && r[4] == 4 && r[5] == 1);
            Wire.responses.push_back(frame(r[1], id, {0, 0, 0, 0, 0, 0, 0x90, 0x40}));
        }
    };
    assert(!c.save_configuration());
    assert(c.get_configuration_field(4, 1) == 1024.0);
}

static void legacy_driver_sleep() {
    struct Case { uint8_t minor, patch, protocol; bool supported; };
    const Case cases[] = {{3, 0, 16, false}, {3, 1, 15, false}, {3, 1, 16, false},
        {3, 0, 17, true}, {3, 1, 17, true}, {3, 1, 18, true}, {3, 2, 17, false},
        {4, 1, 17, false}, {99, 0, 17, false}, {1, 0, 17, false}, {2, 0, 17, false}, {4, 0, 15, true}};
    for (const auto& item : cases) {
        reset(item.minor, item.protocol, item.patch);
        KinisiController c; assert(c.begin(1000, 0));
        assert(c.supportsDriverSleep() == item.supported);
        const auto before = Wire.transactions;
        if (!item.supported) {
            assert(!c.set_driver_sleep(1)); c.get_driver_sleep();
            assert(c.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND && Wire.transactions == before);
            continue;
        }
        uint8_t sleeping = 0;
        Wire.responder = [&](const std::vector<uint8_t>& r) {
            const uint16_t id = r[2] | uint16_t(r[3]) << 8;
            if (r[1] == KINISI_SET_DRIVER_SLEEP) {
                assert(r.size() == 5 && r[4] <= 1);
                sleeping = r[4]; Wire.responses.push_back(frame(r[1], id));
            } else {
                assert(r[1] == KINISI_GET_DRIVER_SLEEP && r.size() == 4);
                Wire.responses.push_back(frame(r[1], id, {sleeping}));
            }
        };
        for (uint8_t requested : {1, 1, 0, 0}) {
            assert(c.set_driver_sleep(requested));
            assert(c.get_driver_sleep() == requested && c.lastError().failure == KinisiFailure::NONE);
        }
        assert(Wire.transactions == before + 8); // No implicit initialization or motion.
        if (item.minor == 3 && item.protocol < 18) assert(!c.supportsBoardIdle());
        Wire.responder = [](const std::vector<uint8_t>& r) {
            Wire.responses.push_back(frame(KINISI_ERROR, r[2] | uint16_t(r[3]) << 8,
                {r[1], uint8_t(KinisiErrorCode::DRIVERS_ASLEEP)}));
        };
        assert(!c.set_driver_sleep(0));
        assert(c.lastError().code == KinisiErrorCode::DRIVERS_ASLEEP && c.ready());
    }
}

static void driver_sleep() {
    reset(4, 15);
    KinisiController c;
    assert(!c.supportsDriverSleep());
    assert(!c.set_driver_sleep(1));
    assert(c.lastError().failure == KinisiFailure::NOT_READY && Wire.transactions == 0);
    c.get_driver_sleep();
    assert(c.lastError().failure == KinisiFailure::NOT_READY && Wire.transactions == 0);
    for (uint8_t board = 1; board <= 5; ++board) {
        reset(board, 15); assert(c.begin(1000, 0));
        assert(c.supportsDriverSleep() == (board == 4));
        if (board == 4) continue;
        const auto before = Wire.transactions;
        assert(!c.set_driver_sleep(1)); c.get_driver_sleep();
        assert(c.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND && Wire.transactions == before);
    }
    reset(4, 14); assert(c.begin(1000, 0));
    auto before = Wire.transactions;
    assert(!c.supportsDriverSleep() && !c.set_driver_sleep(1)); c.get_driver_sleep();
    assert(c.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND && Wire.transactions == before);
    reset(4, 15, 1); assert(c.begin(1000, 0));
    before = Wire.transactions;
    assert(!c.supportsDriverSleep() && !c.set_driver_sleep(1)); c.get_driver_sleep();
    assert(c.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND && Wire.transactions == before);
    reset(4, 15); assert(c.begin(1000, 0));
    uint8_t sleeping = 0;
    bool wake_failure = false;
    auto sleep_response = [&](const std::vector<uint8_t>& r) {
        const uint16_t id = r[2] | uint16_t(r[3]) << 8;
        if (r[1] == KINISI_SET_DRIVER_SLEEP) {
            assert(r.size() == 5 && r[4] <= 1);
            if (!r[4] && wake_failure) {
                Wire.responses.push_back(frame(KINISI_ERROR, id, {r[1], uint8_t(KinisiErrorCode::INTERNAL_ERROR)}));
                return;
            }
            sleeping = r[4];
            Wire.responses.push_back(frame(r[1], id));
        } else if (r[1] == KINISI_GET_DRIVER_SLEEP) {
            assert(r.size() == 4);
            Wire.responses.push_back(frame(r[1], id, {sleeping}));
        } else {
            assert(r[1] == KINISI_SET_MOTOR_SPEED);
            Wire.responses.push_back(frame(KINISI_ERROR, id, {r[1], uint8_t(KinisiErrorCode::DRIVERS_ASLEEP)}));
        }
    };
    Wire.responder = sleep_response;
    assert(c.set_driver_sleep(1) && c.set_driver_sleep(1));
    assert(c.get_driver_sleep() == 1 && c.lastError().failure == KinisiFailure::NONE);
    before = Wire.transactions;
    assert(!c.set_driver_sleep(2) && c.lastError().failure == KinisiFailure::INVALID_ARGUMENT);
    assert(!c.set_driver_sleep(255) && Wire.transactions == before);
    assert(!c.set_motor_speed(0, 10));
    assert(c.lastError().code == KinisiErrorCode::DRIVERS_ASLEEP && c.ready());
    wake_failure = true;
    assert(!c.set_driver_sleep(0) && c.lastError().code == KinisiErrorCode::INTERNAL_ERROR);
    assert(c.get_driver_sleep() == 1);
    wake_failure = false;
    assert(c.set_driver_sleep(0) && c.get_driver_sleep() == 0);
    sleeping = 2;
    c.get_driver_sleep();
    assert(c.lastError().failure == KinisiFailure::MALFORMED_RESPONSE);
    sleeping = 1;
    reset(4, 15);
    Wire.responder = [&](const std::vector<uint8_t>& r) {
        assert(r[1] != KINISI_SET_DRIVER_SLEEP);  // Reconnect never wakes the board.
        if (r[1] == KINISI_GET_DRIVER_SLEEP) sleep_response(r);
        else respond(r);
    };
    assert(c.begin(1000, 0) && c.supportsDriverSleep());
    assert(c.get_driver_sleep() == 1 && c.lastError().failure == KinisiFailure::NONE);
}

static void board_idle(uint8_t idle_revision = 4, uint8_t idle_patch = 0, uint8_t idle_protocol = 16) {
    reset(4, 16);
    KinisiController c;
    assert(!c.supportsBoardIdle());
    assert(!c.set_board_idle(1));
    assert(c.lastError().failure == KinisiFailure::NOT_READY && Wire.transactions == 0);
    c.get_board_idle();
    assert(c.lastError().failure == KinisiFailure::NOT_READY && Wire.transactions == 0);
    for (uint8_t board = 1; board <= 5; ++board) {
        reset(board, 16); assert(c.begin(1000, 0));
        assert(c.supportsBoardIdle() == (board == 4));
        if (board == 4) continue;
        const auto before = Wire.transactions;
        assert(!c.set_board_idle(1)); c.get_board_idle();
        assert(c.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND && Wire.transactions == before);
    }
    reset(4, 15); assert(c.begin(1000, 0));
    auto before = Wire.transactions;
    assert(!c.supportsBoardIdle() && !c.set_board_idle(1)); c.get_board_idle();
    assert(c.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND && Wire.transactions == before);
    reset(4, 16, 1); assert(c.begin(1000, 0));
    before = Wire.transactions;
    assert(!c.supportsBoardIdle() && !c.set_board_idle(1)); c.get_board_idle();
    assert(c.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND && Wire.transactions == before);
    struct IdleGate { uint8_t minor, patch, protocol; };
    for (const IdleGate& gate : {IdleGate{3, 0, 17}, IdleGate{3, 1, 17},
            IdleGate{3, 2, 18}, IdleGate{4, 1, 18}, IdleGate{1, 0, 18}, IdleGate{2, 0, 18}}) {
        reset(gate.minor, gate.protocol, gate.patch); assert(c.begin(1000, 0));
        before = Wire.transactions;
        assert(!c.supportsBoardIdle() && !c.set_board_idle(1)); c.get_board_idle();
        assert(c.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND && Wire.transactions == before);
        if (gate.minor == 3 && gate.patch <= 1) assert(c.supportsDriverSleep());
    }
    reset(idle_revision, idle_protocol, idle_patch); assert(c.begin(1000, 0));
    assert(c.supportsBoardIdle());
    uint8_t sleeping = 0;
    bool wake_failure = false;
    auto sleep_response = [&](const std::vector<uint8_t>& r) {
        const uint16_t id = r[2] | uint16_t(r[3]) << 8;
        if (r[1] == KINISI_SET_BOARD_IDLE) {
            assert(r.size() == 5 && r[4] <= 1);
            if (!r[4] && wake_failure) {
                Wire.responses.push_back(frame(KINISI_ERROR, id, {r[1], uint8_t(KinisiErrorCode::INTERNAL_ERROR)}));
                return;
            }
            sleeping = r[4];
            Wire.responses.push_back(frame(r[1], id));
        } else if (r[1] == KINISI_GET_BOARD_IDLE) {
            assert(r.size() == 4);
            Wire.responses.push_back(frame(r[1], id, {sleeping}));
        } else {
            assert(r[1] == KINISI_SET_DRIVER_SLEEP);
            Wire.responses.push_back(frame(KINISI_ERROR, id, {r[1], uint8_t(KinisiErrorCode::BOARD_IDLE)}));
        }
    };
    Wire.responder = sleep_response;
    assert(c.set_board_idle(1) && c.set_board_idle(1));
    assert(c.get_board_idle() == 1 && c.lastError().failure == KinisiFailure::NONE);
    before = Wire.transactions;
    assert(!c.set_board_idle(2) && c.lastError().failure == KinisiFailure::INVALID_ARGUMENT);
    assert(!c.set_board_idle(255) && Wire.transactions == before);
    assert(!c.set_driver_sleep(0));
    assert(c.lastError().code == KinisiErrorCode::BOARD_IDLE && c.ready());
    wake_failure = true;
    assert(!c.set_board_idle(0) && c.lastError().code == KinisiErrorCode::INTERNAL_ERROR);
    assert(c.get_board_idle() == 1);
    wake_failure = false;
    assert(c.set_board_idle(0) && c.get_board_idle() == 0);
    sleeping = 2;
    c.get_board_idle();
    assert(c.lastError().failure == KinisiFailure::MALFORMED_RESPONSE);
    sleeping = 1;
    reset(idle_revision, idle_protocol, idle_patch);
    Wire.responder = [&](const std::vector<uint8_t>& r) {
        assert(r[1] != KINISI_SET_BOARD_IDLE);  // Reconnect never wakes the board.
        if (r[1] == KINISI_GET_BOARD_IDLE) sleep_response(r);
        else respond(r);
    };
    assert(c.begin(1000, 0) && c.supportsBoardIdle());
    assert(c.get_board_idle() == 1 && c.lastError().failure == KinisiFailure::NONE);
}

static void fusion_experiment() {
    const double pi = 3.14159265358979323846;
    uint8_t encoded[8];
    for (int sign : {-1, 1}) {
        const float endpoint = sign * static_cast<float>(pi);
        kinisi_codec::writeFusionYaw(encoded, endpoint);
        assert(kinisi_codec::readUnsigned(encoded, 8) == (UINT64_C(0x400921fb54442d18) |
            (sign < 0 ? UINT64_C(0x8000000000000000) : 0)));
        const float outside = nextafterf(endpoint, sign * INFINITY);
        kinisi_codec::writeFusionYaw(encoded, outside);
        assert(fabs(kinisi_codec::readDouble(encoded)) > pi);
        // Host binary64 must not normalize widened float endpoints or adjacent doubles.
        kinisi_codec::writeFusionYaw(encoded, static_cast<double>(endpoint));
        assert(fabs(kinisi_codec::readDouble(encoded)) > pi);
        kinisi_codec::writeFusionYaw(encoded, sign * pi);
        assert(kinisi_codec::readDouble(encoded) == sign * pi);
    }
    struct Gate { uint8_t minor, patch, version; bool supported; };
    for (const Gate& gate : {Gate{4,0,19,true}, Gate{4,0,20,true}, Gate{4,0,18,false},
            Gate{4,1,19,false}, Gate{3,0,19,false}, Gate{3,1,19,false}, Gate{99,0,19,false}}) {
        reset(gate.minor, gate.version, gate.patch);
        KinisiController c;
        assert(!c.supportsFusionExperiment());
        assert(c.begin(1000, 0) && c.supportsFusionExperiment() == gate.supported);
        auto before = Wire.transactions;
        if (!gate.supported) {
            assert(!c.set_fusion_experiment(2, 0));
            c.get_fusion_status(); c.get_fusion_vector(0); c.get_fusion_configuration();
            assert(c.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND && Wire.transactions == before);
            continue;
        }
        bool reject = false, malformed = false;
        double expectedYaw = -.5;
        Wire.responder = [&](const std::vector<uint8_t>& r) {
            const uint16_t id = r[2] | uint16_t(r[3]) << 8;
            assert(r[1] >= KINISI_SET_FUSION_EXPERIMENT && r[1] <= KINISI_GET_FUSION_CONFIGURATION);
            if (reject) { Wire.responses.push_back(frame(KINISI_ERROR, id, {r[1], uint8_t(KinisiErrorCode::INVALID_ARGUMENT)})); return; }
            std::vector<uint8_t> data;
            if (r[1] == KINISI_SET_FUSION_EXPERIMENT) {
                assert(r.size() == 13 && r[4] == 2 && kinisi_codec::readDouble(r.data()+5) == expectedYaw);
            } else if (r[1] == KINISI_GET_FUSION_STATUS) {
                data.resize(28); data[0]=malformed ? 3 : 2; data[1]=2; data[3]=3;
                kinisi_codec::writeUnsigned(data.data()+4, 42, 4);
                kinisi_codec::writeUnsigned(data.data()+8, (UINT64_C(1)<<60)+7, 8);
                kinisi_codec::writeUnsigned(data.data()+24, 30, 4);
            } else if (r[1] == KINISI_GET_FUSION_VECTOR) {
                assert(r.size() == 5 && r[4] == 2); data.resize(28);
                kinisi_codec::writeUnsigned(data.data(),42,4);
                kinisi_codec::writeDouble(data.data()+4,1.25);
                kinisi_codec::writeDouble(data.data()+12,-2);
                kinisi_codec::writeDouble(data.data()+20,.5);
            } else {
                data.resize(9);data[0]=2;kinisi_codec::writeDouble(data.data()+1,-.5);
            }
            assert(data.size()+4 <= 32); Wire.responses.push_back(frame(r[1],id,data));
        };
        assert(c.set_fusion_experiment(KinisiFusionMode::FUSED,-.5));
        for (int sign : {-1, 1}) {
            expectedYaw = sign * pi;
            assert(c.set_fusion_experiment(2, expectedYaw));
            const auto count = Wire.transactions;
            assert(!c.set_fusion_experiment(2, nextafter(expectedYaw, sign * INFINITY)));
            assert(!c.set_fusion_experiment(2, nextafterf(static_cast<float>(expectedYaw), sign * INFINITY)));
            assert(!c.set_fusion_experiment(2, static_cast<float>(expectedYaw)));
            assert(c.lastError().failure == KinisiFailure::INVALID_ARGUMENT && Wire.transactions == count);
        }
        auto status = c.get_fusion_status();
        assert(c.lastError().failure == KinisiFailure::NONE && status.sequence == 42);
        assert(status.timestamp_us == (UINT64_C(1)<<60)+7 && status.encoder_age_us == 30);
        auto vector = c.get_fusion_vector(KinisiFusionVectorKind::FUSED_POSE);
        assert(vector.sequence == 42 && vector.x == 1.25 && vector.y == -2 && vector.z == .5);
        assert(c.get_fusion_configuration().mounting_yaw_rad == -.5);
        before = Wire.transactions;
        assert(!c.set_fusion_experiment(3,0)); assert(!c.set_fusion_experiment(2,4));
        c.get_fusion_vector(5);
        assert(c.lastError().failure == KinisiFailure::INVALID_ARGUMENT && Wire.transactions == before);
        reject = true; before = Wire.transactions;
        assert(!c.set_fusion_experiment(2,0));
        assert(c.lastError().code == KinisiErrorCode::INVALID_ARGUMENT && c.ready() && Wire.transactions == before+1);
        reject = false; malformed = true; c.get_fusion_status();
        assert(c.lastError().failure == KinisiFailure::MALFORMED_RESPONSE);
    }
}

static void imu_tracking() {
    for (uint8_t version : {uint8_t(19), uint8_t(20), uint8_t(21)}) {
        reset(4, version); KinisiController c;
        assert(!c.supportsImuTracking()); assert(c.begin(1000, 0));
        assert(c.supportsImuTracking() == (version >= 20));
        auto before = Wire.transactions;
        if (version < 20) {
            assert(!c.set_imu_tracking(0,1,0)); c.get_imu_tracking_status(); c.get_imu_tracking_vector(0); c.get_imu_tracking_configuration();
            assert(c.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND && Wire.transactions == before); continue;
        }
        bool reject = false, malformed = false;
        Wire.responder = [&](const std::vector<uint8_t>& r) {
            uint16_t id=r[2] | uint16_t(r[3])<<8; std::vector<uint8_t> data;
            if (reject) { Wire.responses.push_back(frame(KINISI_ERROR,id,{r[1],uint8_t(KinisiErrorCode::INVALID_ARGUMENT)})); return; }
            if (r[1]==KINISI_SET_IMU_TRACKING) {
                assert(r.size()==14 && r[4]<=3);
                assert(r[4]==1 ? r[5]==0 && kinisi_codec::readDouble(r.data()+6)==0 : r[5]==1 && kinisi_codec::readDouble(r.data()+6)==3.141592653589793);
            } else if (r[1]==KINISI_GET_IMU_TRACKING_STATUS) {
                data.resize(28); data[0]=malformed?5:2; data[2]=3;
                kinisi_codec::writeUnsigned(data.data()+4,42,4); kinisi_codec::writeUnsigned(data.data()+8,(UINT64_C(1)<<60)+7,8);
                kinisi_codec::writeUnsigned(data.data()+16,500,4); kinisi_codec::writeUnsigned(data.data()+20,10,4); kinisi_codec::writeUnsigned(data.data()+24,20,4);
            } else if (r[1]==KINISI_GET_IMU_TRACKING_VECTOR) {
                data.resize(28); kinisi_codec::writeUnsigned(data.data(),42,4); kinisi_codec::writeDouble(data.data()+4,1.25);
            } else if (r[1]==KINISI_GET_IMU_TRACKING_CONFIGURATION) {
                data.resize(12); kinisi_codec::writeDouble(data.data(),3.141592653589793); kinisi_codec::writeUnsigned(data.data()+8,5000000,4);
            } else assert(false);
            assert(data.size()+4<=32); Wire.responses.push_back(frame(r[1],id,data));
        };
        for (auto action : {KinisiImuTrackingAction::START,KinisiImuTrackingAction::RESET,KinisiImuTrackingAction::RECALIBRATE})
            assert(c.set_imu_tracking(action,1,3.141592653589793));
        assert(c.set_imu_tracking(KinisiImuTrackingAction::STOP,0,0));
        auto status=c.get_imu_tracking_status(); assert(status.timestamp_us==(UINT64_C(1)<<60)+7 && status.elapsed_us==500 && status.accel_age_us==10 && status.gyro_age_us==20);
        assert(c.get_imu_tracking_vector(KinisiImuTrackingVectorKind::POSE).x==1.25);
        assert(c.get_imu_tracking_configuration().window_us==5000000);
        before=Wire.transactions;
        assert(!c.set_imu_tracking(0,0,0)); assert(!c.set_imu_tracking(1,1,0)); assert(!c.set_imu_tracking(1,0,1)); assert(!c.set_imu_tracking(4,1,0));
        assert(!c.set_imu_tracking(0,1,INFINITY)); assert(!c.set_imu_tracking(0,1,NAN)); assert(!c.set_imu_tracking(0,1,4));
        c.get_imu_tracking_vector(3); assert(c.lastError().failure==KinisiFailure::INVALID_ARGUMENT && Wire.transactions==before);
        reject=true; assert(!c.set_imu_tracking(0,1,0)); assert(c.ready() && c.lastError().code==KinisiErrorCode::INVALID_ARGUMENT && Wire.transactions==before+1);
        reject=false; malformed=true; c.get_imu_tracking_status(); assert(c.lastError().failure==KinisiFailure::MALFORMED_RESPONSE);
    }
    for (uint8_t minor : {uint8_t(1),uint8_t(2),uint8_t(3),uint8_t(99)}) {
        reset(minor,20); KinisiController c; assert(c.begin(1000,0)); assert(!c.supportsImuTracking());
        auto before=Wire.transactions; c.get_imu_tracking_status(); assert(Wire.transactions==before);
    }
    assert(legacy_capabilities_v219.fusion_experiment && !legacy_capabilities_v219.imu_tracking);
    for (uint64_t sign : {UINT64_C(0), UINT64_C(0x8000000000000000)}) {
        reset(4,20); KinisiController client; assert(client.begin(1000,0));
        uint64_t bits = sign | UINT64_C(0x400921fb54442d18);
        Wire.responder = [&](const std::vector<uint8_t>& request) {
            std::vector<uint8_t> config(12);
            kinisi_codec::writeUnsigned(config.data(),bits,8);
            kinisi_codec::writeUnsigned(config.data()+8,5000000,4);
            Wire.responses.push_back(frame(request[1],request[2] | uint16_t(request[3])<<8,config));
        };
        assert(client.get_imu_tracking_configuration().window_us==5000000);
        ++bits;
        client.get_imu_tracking_configuration();
        assert(client.lastError().failure==KinisiFailure::MALFORMED_RESPONSE && !client.ready());
    }
}

static void feedback_source_selector() {
    reset(4, 21);
    KinisiController c;
    assert(!c.supportsFeedbackSourceSelector());
    assert(c.begin(1000, 0) && c.supportsFeedbackSourceSelector());
    uint8_t source = 0;
    uint32_t origin = 3, sequence = 42, captured_age = 937, confirmed_age = 937;
    uint64_t stamp = (UINT64_C(1)<<60)+7;
    uint32_t next_sequence = 42;
    int32_t tick = 0;
    uint16_t limit_override = 0;
    unsigned reads = 0;
    bool changing = false, expired = false, unsafe = false, malformed = false;
    auto count = Wire.transactions;
    Wire.responder = [&](const std::vector<uint8_t>& request) {
        const uint8_t cmd = request[1];
        const uint16_t id = request[2] | uint16_t(request[3]) << 8;
        std::vector<uint8_t> data;
        if (cmd == KINISI_GET_FEEDBACK_SOURCE_STATUS) {
            data.resize(27);
            const bool confirming = ++reads % 2 == 0;
            data[0] = source; data[1] = expired ? 255 : source; data[2] = expired ? 0 : 1;
            data[3] = malformed ? 11 : expired ? 5 : 0;
            kinisi_codec::writeUnsigned(data.data()+4, expired ? 0 : confirming ? next_sequence : sequence, 4);
            kinisi_codec::writeUnsigned(data.data()+8, expired ? 0 : confirming ? stamp + tick : stamp, 8);
            kinisi_codec::writeUnsigned(data.data()+16, changing ? ++origin : origin, 4);
            kinisi_codec::writeUnsigned(data.data()+21, expired ? UINT32_MAX : confirming ? confirmed_age : captured_age, 4);
            const uint16_t max_age = source == 0 ? 300 : source == 1 ? 50 : 60;
            kinisi_codec::writeUnsigned(data.data()+25, confirming && limit_override ? limit_override : max_age, 2);
        } else if (cmd == KINISI_GET_FEEDBACK_SOURCE_VECTOR) {
            assert(request.size() == 13 && request[4] < 2);
            assert(kinisi_codec::readUnsigned(request.data()+5, 4) == origin && kinisi_codec::readUnsigned(request.data()+9, 4) == sequence);
            data.resize(28); kinisi_codec::writeUnsigned(data.data(), sequence, 4);
            kinisi_codec::writeDouble(data.data()+4, request[4] == 0 ? 1.25 : -0.5);
        } else if (cmd == KINISI_SET_FEEDBACK_SOURCE && unsafe) {
            Wire.responses.push_back(frame(KINISI_ERROR, id, {cmd, uint8_t(KinisiErrorCode::SOURCE_SWITCH_UNSAFE)}));
            return;
        } else if (cmd == KINISI_SET_FEEDBACK_FUSION_MOUNT) {
            assert(request.size() == 12 && kinisi_codec::readDouble(request.data()+4) == -3.141592653589793);
        } else if (cmd == KINISI_GET_FEEDBACK_FUSION_MOUNT) {
            data.resize(9); data[0] = 1; kinisi_codec::writeDouble(data.data()+1, -3.141592653589793);
        } else assert(false);
        assert(data.size()+4 <= 32);
        Wire.responses.push_back(frame(cmd, id, data));
    };
    feedback_source_status status{};
    feedback_source_vector pose{}, twist{};
    for (source = 0; source < 3; ++source) {
        assert(c.readFeedbackSourceSnapshot(status, pose, twist) == KinisiFeedbackRead::VALID);
        assert(status.requested == source && status.active == source && status.valid && status.origin_id == origin &&
            status.sequence == 42 && status.timestamp_us == (UINT64_C(1)<<60)+7 && status.age_us == 937);
        assert(pose.x == 1.25 && twist.x == -0.5);
    }
    source = 2; changing = true;
    assert(c.readFeedbackSourceSnapshot(status, pose, twist) == KinisiFeedbackRead::CHANGED);
    assert(pose.x == 0 && twist.x == 0 && !status.valid);
    assert(Wire.transactions == count + 24);
    changing = false;
    sequence = UINT32_MAX; next_sequence = 0; tick = 10000;
    for (source = 0; source < 3; ++source) {
        captured_age = confirmed_age = source == 0 ? 150000 : 1000;
        assert(c.readFeedbackSourceSnapshot(status, pose, twist) == KinisiFeedbackRead::VALID);
        assert(status.sequence == sequence && status.timestamp_us == stamp && status.age_us == captured_age + 10000);
    }
    sequence = 42;
    struct BoundsCase { uint8_t source; uint32_t first_age; int32_t delta; uint32_t last_age, seq; uint16_t max_age; };
    const BoundsCase rejected[] = {{1,40000,12000,40000,43,50}, {0,150000,60000,150000,43,300},
        {0,50000,12000,49500,43,60}, {2,1000,-1,1000,43,60}, {2,1000,1000,1000,41,60}};
    for (const auto& trial : rejected) {
        source = trial.source; captured_age = trial.first_age; tick = trial.delta;
        confirmed_age = trial.last_age; next_sequence = trial.seq; limit_override = trial.max_age;
        assert(c.readFeedbackSourceSnapshot(status, pose, twist) == KinisiFeedbackRead::CHANGED);
    }
    source = 2; expired = true;
    assert(c.readFeedbackSourceSnapshot(status, pose, twist) == KinisiFeedbackRead::UNAVAILABLE);
    assert(!status.valid && status.active == 255 && status.reason == 5 && status.age_us == UINT32_MAX);
    unsafe = true; count = Wire.transactions;
    assert(!c.set_feedback_source(KinisiFeedbackSource::IMU_ONLY) && c.lastError().code == KinisiErrorCode::SOURCE_SWITCH_UNSAFE);
    assert(Wire.transactions == count + 1 && c.ready());
    assert(!c.set_feedback_source(KinisiFeedbackSource::NONE));
    c.get_feedback_source_vector(2, 3, 42);
    assert(!c.set_feedback_fusion_mount(NAN) && !c.set_feedback_fusion_mount(INFINITY));
    assert(Wire.transactions == count + 1 && c.lastError().failure == KinisiFailure::INVALID_ARGUMENT);
    assert(c.set_feedback_fusion_mount(-3.141592653589793));
    assert(c.get_feedback_fusion_mount().confirmed == 1);
    malformed = true;
    c.get_feedback_source_status();
    assert(c.lastError().failure == KinisiFailure::MALFORMED_RESPONSE);
    for (uint8_t minor : {uint8_t(3),uint8_t(4)}) {
        reset(minor, minor == 4 ? 20 : 21);
        assert(c.begin(1000, 0) && !c.supportsFeedbackSourceSelector());
        count = Wire.transactions;
        assert(!c.set_feedback_source(0)); c.get_feedback_source_status(); c.get_feedback_source_vector(0, 3, 42);
        assert(!c.set_feedback_fusion_mount(0)); c.get_feedback_fusion_mount();
        assert(c.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND && Wire.transactions == count);
    }
}

int main() {
    if (KINISI_WIRE_BUFFER_SIZE < 32) {
        reset(4, 21);
        KinisiController controller;
        assert(controller.begin(1000, 0) && !controller.supportsFeedbackSourceSelector());
        const auto transactions = Wire.transactions;
        assert(!controller.set_feedback_source(KinisiFeedbackSource::IMU_ONLY));
        controller.get_feedback_source_status();
        controller.get_feedback_source_vector(KinisiFeedbackSourceVectorKind::POSE, 3, 42);
        assert(!controller.set_feedback_fusion_mount(0));
        controller.get_feedback_fusion_mount();
        assert(Wire.transactions == transactions && controller.lastError().failure == KinisiFailure::UNSUPPORTED_COMMAND);
        puts("PASS selector withheld when Wire cannot carry a complete vector");
        return 0;
    }
    feedback_source_selector();
    imu_tracking();
    fusion_experiment();
    legacy_driver_sleep();
    board_idle();
    board_idle(3, 0, 18);
    board_idle(3, 1, 18);
    board_idle(3, 1, 19);
    driver_sleep();
    configuration_save();
    pwm_frequencies();
    current_limits();
    clear_faults();
    profiles(); sensor_reads_and_capacity(); gating_and_unknown_boards(); errors_and_lifecycle(); invalid_samples();
    current_readings(); fault_readings();
    printf("PASS board profiles, sensors and Wire capacity %u\n", unsigned(KINISI_WIRE_BUFFER_SIZE));
}
