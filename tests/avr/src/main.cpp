// File: tests/avr/src/main.cpp
// Compile-check the public API with the real AVR Wire core; never enables motors.
#include <Arduino.h>
#include <kinisi.h>
#include <kinisi_codec.h>
#include "../../legacy_profile.h"
KinisiController controller;

/** Check AVR numeric conversions against fixed binary64 bytes on target startup. */
void setup() {
    Serial.begin(9600);
    if (!legacy_profile.hasSensor(KinisiSensor::IMU) || legacy_profile.power.driver_sleep ||
            !legacy_profile_v215.power.driver_sleep || legacy_profile_v215.power.board_idle) {
        Serial.println("FAIL legacy aggregate profile");
        return;
    }
    uint8_t bytes[8];
    kinisi_codec::writeDouble(bytes, -12.5);
    if (kinisi_codec::readUnsigned(bytes, 8) != UINT64_C(0xc029000000000000)) {
        Serial.println("FAIL binary64 encoding");
        return;
    }
    kinisi_codec::writeUnsigned(bytes, UINT64_C(0x3ff4000000000000), 8);
    if (kinisi_codec::readDouble(bytes) != 1.25) {
        Serial.println("FAIL binary64 decoding");
        return;
    }
    for (int8_t sign = -1; sign <= 1; sign += 2) {
        kinisi_codec::writeFusionYaw(bytes, sign * PI);
        if (kinisi_codec::readUnsigned(bytes, 8) != (UINT64_C(0x400921fb54442d18) |
                (sign < 0 ? UINT64_C(0x8000000000000000) : 0))) {
            Serial.println("FAIL yaw endpoint encoding");
            return;
        }
    }
    KinisiBoardProfile profile;
    if (!kinisiLookupBoardProfile(1, 0, 4, 0, profile) || profile.resources.motors != 4 ||
            !profile.hasSensor(KinisiSensor::IMU) || !profile.power.driver_sleep) {
        Serial.println("FAIL bundled flash profile");
        return;
    }
    if (!controller.begin()) {
        Serial.println("Controller unavailable");
        return;
    }
    const init_response& board = controller.boardInfo();
    Serial.println(board.board_minor);
    time_status clock = controller.get_time_status();
    if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(clock.clock_mode);
    if (controller.boardProfile()) Serial.println(controller.boardProfile()->resources.encoders);
    if (controller.supportsSensor(KinisiSensor::TEMPERATURE)) {
        double temperature = controller.get_temperature(0);
        if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(temperature);
    }
    // Compiles the IMU getter too; a default Uno rejects this before I2C (32-byte Wire).
    imu_sample sample = controller.get_imu(0);
    if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(sample.acceleration_z);
    if (controller.supportsTopic(KinisiTopic::TEMPERATURE))
        controller.subscribe(KinisiTopic::TEMPERATURE, 0, 250);
    if (controller.supportsMotorFaults(0)) {
        motor_fault_status status = controller.get_motor_faults(0);
        if (controller.boardInfo().protocol_minor >= 10) status = controller.clear_motor_faults(0);
        if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(status.status_valid);
        if (controller.boardInfo().protocol_minor >= 12)
            controller.subscribe(KinisiTopic::MOTOR_FAULTS, 0, 0, KinisiSubscriptionTrigger::ON_EVENT, KinisiSubscriptionDelivery::ON_CHANGE, 5000);
        else controller.subscribe(KinisiTopic::MOTOR_FAULTS, 0, 100, KinisiSubscriptionMode::ON_CHANGE);
    }
    if (controller.supportsPwmFrequency()) {
        uint32_t frequency = controller.set_pwm_frequency(KinisiPwmFrequency::KHZ_25);
        if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(frequency);
        frequency = controller.get_pwm_frequency();
        if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(frequency);
    }
    if (controller.supportsMotorCurrentLimit(0)) {
        const KinisiCurrentLimitOptions* options = controller.getMotorCurrentLimitOptions(0);
        motor_current_limit setting = controller.get_motor_current_limit(0);
        if (options) setting = controller.set_motor_current_limit(0, true, options->supported_thresholds_ma[0]);
        if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(setting.threshold_ma);
    }
    if (controller.supportsMotorCurrent(0)) {
        double amps = controller.get_motor_current(0);
        if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(amps);
        controller.subscribe(KinisiTopic::MOTOR_CURRENT, 0, 100);
    }
    if (controller.supportsFeedbackSourceSelector()) {
        feedback_source_status status{};
        feedback_source_vector pose{}, twist{};
        if (controller.readFeedbackSourceSnapshot(status, pose, twist) == KinisiFeedbackRead::VALID) {
            Serial.println(status.sequence);
            Serial.println(pose.x);
        }
    }
    if (controller.supportsImuTracking()) {
        imu_tracking_status status = controller.get_imu_tracking_status();
        if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(status.elapsed_us);
        imu_tracking_vector pose = controller.get_imu_tracking_vector(KinisiImuTrackingVectorKind::POSE);
        if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(pose.x);
        imu_tracking_configuration configuration = controller.get_imu_tracking_configuration();
        if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(configuration.window_us);
        controller.set_imu_tracking(KinisiImuTrackingAction::STOP, 0, 0);
    }
    if (controller.supportsFusionExperiment()) {
        fusion_configuration configuration = controller.get_fusion_configuration();
        if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(configuration.mode);
        fusion_status status = controller.get_fusion_status();
        if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(status.sequence);
        fusion_vector pose = controller.get_fusion_vector(KinisiFusionVectorKind::FUSED_POSE);
        if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(pose.x);
        // Explicit legacy selection is compile checked only; never commands motion.
        controller.set_fusion_experiment(KinisiFusionMode::LEGACY_ENCODER, 0.0);
    }
    if (controller.supportsDriverSleep()) {
        if (controller.set_driver_sleep(1)) {
            uint8_t sleeping = controller.get_driver_sleep();
            if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(sleeping);
        }
    }
    if (controller.supportsBoardIdle()) {
        if (controller.set_board_idle(1)) {
            uint8_t idle = controller.get_board_idle();
            if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(idle);
        }
    }
}

/** Compile the cooperative heartbeat service without enabling motors. */
void loop() {
    controller.poll();
    KinisiTelemetrySample sample;
    if (controller.getSubscriptionSample(KinisiTopic::TEMPERATURE, 0, sample))
        Serial.println(sample.data.temperature.celsius);
    if (controller.getSubscriptionSample(KinisiTopic::MOTOR_FAULTS, 0, sample))
        Serial.println(sample.data.motor_faults.overcurrent);
    if (controller.getSubscriptionSample(KinisiTopic::MOTOR_CURRENT, 0, sample))
        Serial.println(sample.data.motor_current.amps);
}
