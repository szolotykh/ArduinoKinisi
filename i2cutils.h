// File: i2cutils.h
// Whole-frame Wire transport. Transactions must fit the actual Wire buffers.
#ifndef I2CUTILS_H
#define I2CUTILS_H
#include <Arduino.h>
#include <Wire.h>

// Override only after configuring the core's real TX and RX buffers to this size.
#ifndef KINISI_WIRE_BUFFER_SIZE
#if defined(BUFFER_LENGTH)
#define KINISI_WIRE_BUFFER_SIZE BUFFER_LENGTH
#elif defined(I2C_BUFFER_LENGTH)
#define KINISI_WIRE_BUFFER_SIZE I2C_BUFFER_LENGTH
#else
#define KINISI_WIRE_BUFFER_SIZE 32
#endif
#endif

/** Start Wire and enable a 100-ms hardware timeout on cores exposing that API. */
void kinisiWireBegin();
/** Send one complete frame; return false on overflow, short write or bus error. */
bool kinisiWireSend(uint8_t address, const uint8_t* frame, uint8_t length);
/** Read one transaction. Return bytes received, or -1 for a bus timeout/error. */
int kinisiWireReceive(uint8_t address, uint8_t* frame, uint8_t length);
#endif
