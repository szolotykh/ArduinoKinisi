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
    if (!controller.begin()) {
        Serial.println("Controller unavailable");
        return;
    }
    const init_response& board = controller.boardInfo();
    Serial.println(board.board_minor);
    time_status clock = controller.get_time_status();
    if (controller.lastError().failure == KinisiFailure::NONE) Serial.println(clock.clock_mode);
}

/** Compile the cooperative heartbeat service without enabling motors. */
void loop() { controller.poll(); }
