// File: tests/mocks/Arduino.h
// Minimal Arduino surface for host-side protocol and example compilation.
#ifndef TEST_ARDUINO_H
#define TEST_ARDUINO_H
#include <stdint.h>
#include <stddef.h>
#include <string>
extern uint32_t fake_millis;
inline unsigned long millis() { return fake_millis; }
inline void delay(unsigned long ms) { fake_millis += uint32_t(ms); }
const int LED_BUILTIN=13, OUTPUT=1, HIGH=1, LOW=0;
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
struct String {
    template<class T> String(T) {}
};
inline String operator+(const char*, const String&) { return String(""); }
inline String operator+(const String&, const char*) { return String(""); }
struct MockSerial {
    void begin(int) {}
    template<class T> void println(T) {}
    template<class T> void print(T) {}
};
extern MockSerial Serial;
#endif
