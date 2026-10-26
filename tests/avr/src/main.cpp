// File: tests/avr/src/main.cpp
// Compile-check the public API with the real AVR Wire core; never enables motors.
#include <Arduino.h>
#include <kinisi.h>
#include <kinisi_codec.h>
KinisiController controller;

/** Check AVR numeric conversions against fixed binary64 bytes on target startup. */
void setup() {
    Serial.begin(9600);
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
    KinisiBoardProfile profile;
    if (!kinisiLookupBoardProfile(1, 0, 4, 0, profile) || profile.resources.motors != 4 ||
            !profile.hasSensor(KinisiSensor::IMU)) {
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
}

/** Compile the cooperative heartbeat service without enabling motors. */
void loop() {
    controller.poll();
    KinisiTelemetrySample sample;
    if (controller.getSubscriptionSample(KinisiTopic::TEMPERATURE, 0, sample))
        Serial.println(sample.data.temperature.celsius);
}
