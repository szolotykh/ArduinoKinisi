// File: tests/mocks/Wire.h
// Transaction-level fake; each response is independently queued by the fixture.
#ifndef TEST_WIRE_H
#define TEST_WIRE_H
#include <Arduino.h>
#include <vector>
#include <deque>
#include <functional>
#include <algorithm>
#define WIRE_HAS_TIMEOUT 1
class TwoWire {
public:
    std::vector<uint8_t> sent;
    std::deque<std::vector<uint8_t> > responses;
    std::function<void(const std::vector<uint8_t>&)> responder;
    std::vector<uint8_t> current;
    size_t cursor=0, transactions=0, reads=0;
    bool nack=false, timeout=false, inject_timeout=false, short_write=false;
    uint32_t bus_timeout=0;
    void begin() {}
    void setWireTimeout(uint32_t us, bool) { bus_timeout=us; }
    void clearWireTimeoutFlag() { timeout=false; }
    bool getWireTimeoutFlag() { return timeout; }
    void beginTransmission(uint8_t address) { if(address != 8) nack=true; sent.clear(); }
    size_t write(const uint8_t* bytes, size_t n) {
        size_t accepted=short_write ? n-1 : n;
        sent.assign(bytes, bytes+accepted);
        return accepted;
    }
    uint8_t endTransmission() {
        ++transactions;
        if(nack) return 2;
        if(responder) responder(sent);
        return 0;
    }
    uint8_t requestFrom(uint8_t, uint8_t n) {
        ++reads; current.clear(); cursor=0;
        if(inject_timeout) { timeout=true; return 0; }
        if(responses.empty()) return 0;
        current=responses.front(); responses.pop_front();
        // Hardware may return padding after a short ACK/ERROR, never another frame.
        current.resize(n, 0xff);
        return n;
    }
    int available() { return int(current.size()-cursor); }
    int read() { return current[cursor++]; }
};
extern TwoWire Wire;
#endif
