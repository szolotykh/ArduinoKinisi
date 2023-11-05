//------------------------------------------------------------
// File name: i2cutils.cpp
//------------------------------------------------------------

#include "i2cutils.h"
#include "Wire.h"

void SendMessage(uint8_t address, uint8_t* message, uint8_t length)
{
    Wire.beginTransmission(address);
    Wire.write(length);
    Wire.write(message, length);
    Wire.endTransmission();
}

void ReceiveResponse(uint8_t address, uint8_t* response, uint8_t length)
{
    Wire.requestFrom(address, length);
    int received = 0;  
    while (received < length)
    {
        if (Wire.available())
        {
            response[received++] = Wire.read();
        }
    }
}

unsigned int DecodeUInt(uint8_t* bytes)
{
    unsigned int result = (unsigned int)bytes[0];
    result |= (unsigned int)bytes[1] << 8;
    result |= (unsigned int)bytes[2] << 16;
    result |= (unsigned int)bytes[3] << 24;
    return result;
}