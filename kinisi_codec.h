// File: kinisi_codec.h
// Packed protocol codecs independent of native alignment, byte order and double width.
#ifndef KINISI_CODEC_H
#define KINISI_CODEC_H
#include <stdint.h>
#include <string.h>
#include <math.h>

namespace kinisi_codec {
/** Read up to eight little-endian bytes without unaligned pointer casts. */
inline uint64_t readUnsigned(const uint8_t* data, uint8_t size) {
    uint64_t value = 0;
    for (uint8_t i = 0; i < size; ++i) value |= uint64_t(data[i]) << (8 * i);
    return value;
}
/** Write the low size bytes of an integer in little-endian order. */
inline void writeUnsigned(uint8_t* data, uint64_t value, uint8_t size) {
    for (uint8_t i = 0; i < size; ++i) data[i] = uint8_t(value >> (8 * i));
}
/** Encode binary64 on the wire, including when AVR double is only binary32. */
inline void writeDouble(uint8_t* data, double value) {
    uint64_t bits;
    if (sizeof(double) == 8) {
        memcpy(&bits, &value, 8);
    } else {
        // frexp uses the board's native precision; binary32 values widen exactly.
        bits = signbit(value) ? (uint64_t(1) << 63) : 0;
        if (isnan(value)) bits |= UINT64_C(0x7ff8000000000000);
        else if (isinf(value)) bits |= UINT64_C(0x7ff0000000000000);
        else if (value != 0) {
            int exponent;
            double fraction = frexp(fabs(value), &exponent);
            bits |= uint64_t(exponent + 1022) << 52;
            bits |= uint64_t(ldexp(fraction * 2 - 1, 52));
        }
    }
    writeUnsigned(data, bits, 8);
}
/** Preserve inclusive yaw endpoints when native binary32 pi rounds above wire pi.
 * Only this bounded angle codec normalizes endpoints; other doubles stay exact.
 * T is deduced from the native setter parameter and can be float in host tests.
 */
template <typename T>
inline void writeFusionYaw(uint8_t* data, T value) {
    const T nativePi = static_cast<T>(3.14159265358979323846);
    if (sizeof(T) == 4 && (value == nativePi || value == -nativePi)) {
        writeUnsigned(data, UINT64_C(0x400921fb54442d18) |
            (value < 0 ? UINT64_C(0x8000000000000000) : 0), 8);
    } else {
        writeDouble(data, static_cast<double>(value));
    }
}
/** Decode binary64; small AVR boards round to their native double precision. */
inline double readDouble(const uint8_t* data) {
    uint64_t bits = readUnsigned(data, 8);
    if (sizeof(double) == 8) {
        double value;
        memcpy(&value, &bits, 8);
        return value;
    }
    uint16_t exponent = uint16_t((bits >> 52) & 0x7ff);
    uint64_t fraction = bits & UINT64_C(0x000fffffffffffff);
    double value;
    if (exponent == 0x7ff) value = fraction ? NAN : INFINITY;
    else if (exponent == 0) value = ldexp(double(fraction), -1074);
    else value = ldexp(1 + ldexp(double(fraction), -52), int(exponent) - 1023);
    return bits >> 63 ? -value : value;
}
}
#endif
