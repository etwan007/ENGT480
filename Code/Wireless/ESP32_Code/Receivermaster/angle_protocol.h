#ifndef ENGT_ANGLE_PROTOCOL_H
#define ENGT_ANGLE_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

namespace AngleProtocol {
static const uint8_t VERSION = 1;
static const uint8_t PAYLOAD_SIZE = 29;
static const size_t FRAME_SIZE = 37;
static const uint32_t PRESENT_TIMEOUT_MS = 50;
enum Status : uint8_t { VALID = 0, ENCODER_FAILED = 1, WIRELESS_TIMEOUT = 2, NEVER_RECEIVED = 3 };

struct Sample {
  uint16_t angle;
  uint32_t receivedMs;
  bool seen;
};

inline uint16_t crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

inline void put16(uint8_t* out, uint16_t value) {
  out[0] = (uint8_t)(value >> 8); out[1] = (uint8_t)value;
}
inline void put32(uint8_t* out, uint32_t value) {
  out[0] = (uint8_t)(value >> 24); out[1] = (uint8_t)(value >> 16);
  out[2] = (uint8_t)(value >> 8); out[3] = (uint8_t)value;
}

// now must be captured AFTER the synchronized copy of samples.
inline void encode(uint8_t* out, uint32_t sequence, uint32_t now, const Sample* samples) {
  out[0] = 0xA5; out[1] = 0x5A; out[2] = 0xC3; out[3] = 0x3C;
  out[4] = VERSION; out[5] = PAYLOAD_SIZE;
  put32(out + 6, sequence); put32(out + 10, now);
  for (size_t i = 0; i < 3; ++i) {
    const Sample& sample = samples[i];
    uint32_t age = sample.seen ? (uint32_t)(now - sample.receivedMs) : UINT32_MAX;
    Status status = !sample.seen ? NEVER_RECEIVED :
        age > PRESENT_TIMEOUT_MS ? WIRELESS_TIMEOUT :
        sample.angle == 0xFFFF ? ENCODER_FAILED : VALID;
    const size_t offset = 14 + i * 7;
    put16(out + offset, status == VALID ? sample.angle : 0xFFFF);
    out[offset + 2] = status;
    put32(out + offset + 3, age);
  }
  put16(out + FRAME_SIZE - 2, crc16(out + 4, FRAME_SIZE - 6));
}
}  // namespace AngleProtocol
#endif
