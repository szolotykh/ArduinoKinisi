// File: i2cutils.cpp
// Wire adapter: never split protocol frames across independent I2C transactions.
#include "i2cutils.h"

/** Initialize the master and bound hardware clock stretching when supported. */
void kinisiWireBegin() {
    Wire.begin();
#ifdef WIRE_HAS_TIMEOUT
    Wire.setWireTimeout(100000UL, true);
    Wire.clearWireTimeoutFlag();
#endif
}
/** Queue a whole frame and verify both the buffer write and I2C acknowledgement. */
bool kinisiWireSend(uint8_t address, const uint8_t* frame, uint8_t length) {
    if (length > KINISI_WIRE_BUFFER_SIZE) return false;
#ifdef WIRE_HAS_TIMEOUT
    Wire.clearWireTimeoutFlag();
#endif
    Wire.beginTransmission(address);
    if (Wire.write(frame, length) != length) {
        // Do not endTransmission: that would send the partially queued frame.
        // The next beginTransmission resets the local TX buffer.
        return false;
    }
    uint8_t result = Wire.endTransmission();
#ifdef WIRE_HAS_TIMEOUT
    if (Wire.getWireTimeoutFlag()) return false;
#endif
    return result == 0;
}
/** Read a bounded buffer; let the protocol layer validate its length and identity. */
int kinisiWireReceive(uint8_t address, uint8_t* frame, uint8_t length) {
    if (length > KINISI_WIRE_BUFFER_SIZE) return -1;
#ifdef WIRE_HAS_TIMEOUT
    Wire.clearWireTimeoutFlag();
#endif
    Wire.requestFrom(address, length);
#ifdef WIRE_HAS_TIMEOUT
    if (Wire.getWireTimeoutFlag()) return -1;
#endif
    uint8_t received = 0;
    while (Wire.available() && received < length) frame[received++] = uint8_t(Wire.read());
    return received;
}
