//------------------------------------------------------------
// File name: i2cutils.h
//------------------------------------------------------------

#ifndef I2CUTILS_H
#define I2CUTILS_H

#include "Arduino.h"

// I2C utils functions
void SendMessage(uint8_t address, uint8_t* message, uint8_t length);
void ReceiveResponse(uint8_t address, uint8_t* response, uint8_t length);

// Decoding functions
unsigned int DecodeUInt(uint8_t* bytes);

#endif // I2CUTILS_H