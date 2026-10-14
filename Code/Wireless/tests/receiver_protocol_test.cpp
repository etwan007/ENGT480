// Host test of the actual receiver sketch with mocked ESP32/serial APIs.
#include <cstdio>
#include <WiFi.h>
uint32_t mockMillis = 0;
int mockLockDepth = 0;
void (*mockBeforeLock)() = nullptr;
MockSerial Serial;
MockWiFi WiFi;
#include "../ESP32_Code/Receivermaster/Receivermaster.ino"

static uint16_t get16(const uint8_t* b) { return (uint16_t)((b[0] << 8) | b[1]); }
static uint32_t get32(const uint8_t* b) {
  return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
}
static void receiveAtNextMillisecond() {
  ++mockMillis;
  AnglePacket packet = {0, 9054};
  onRecv(nullptr, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}

int main() {
  using namespace AngleProtocol;
  assert(crc16(reinterpret_cast<const uint8_t*>("123456789"), 9) == 0x29B1);
  // Reproduce arrival between the scheduler's clock read and sample snapshot.
  mockMillis = 1000;
  mockBeforeLock = receiveAtNextMillisecond;
  loop();
  assert(Serial.frames.size() == 1);
  const auto& first = Serial.frames[0];
  assert(get32(first.data() + 10) == 1001);
  assert(get16(first.data() + 14) == 9054);
  assert(first[16] == VALID && get32(first.data() + 17) == 0);
  assert(first[23] == NEVER_RECEIVED && first[30] == NEVER_RECEIVED);
  assert(mockLockDepth == 0);

  // No catch-up burst after a delayed main loop.
  mockMillis = 2000;
  loop(); loop(); loop();
  assert(Serial.frames.size() == 2);
  assert(Serial.frames.back()[16] == WIRELESS_TIMEOUT);

  Sample samples[3] = {{9054, 950, true}, {0xFFFF, 999, true}, {0, 0, false}};
  uint8_t frame[FRAME_SIZE];
  encode(frame, 0x01020304, 1000, samples);
  assert(frame[16] == VALID && get32(frame + 17) == 50);
  assert(frame[23] == ENCODER_FAILED && get32(frame + 24) == 1);
  assert(frame[30] == NEVER_RECEIVED && get32(frame + 31) == UINT32_MAX);
  assert(get16(frame + 35) == crc16(frame + 4, 31));
  // Print a cross-language fixture emitted by the real firmware encoder.
  for (size_t i = 0; i < FRAME_SIZE; ++i) std::printf("%02x", frame[i]);
  std::printf("\n");
  encode(frame, 1, 1001, samples);
  assert(frame[16] == WIRELESS_TIMEOUT);
  // A reading at boot time zero is present; millis wrap remains valid.
  samples[0] = {1234, 0, true};
  encode(frame, 2, 10, samples);
  assert(frame[16] == VALID && get32(frame + 17) == 10);
  samples[0] = {1234, 0xFFFFFFFCu, true};
  encode(frame, 3, 3, samples);
  assert(frame[16] == VALID && get32(frame + 17) == 7);
  // Malformed radio angle/id must not poison the latest valid sample.
  AnglePacket bad = {0, 40000};
  onRecv(nullptr, reinterpret_cast<const uint8_t*>(&bad), sizeof(bad));
  assert(g_samples[0].angle == 9054);
  bad = {3, 42};
  onRecv(nullptr, reinterpret_cast<const uint8_t*>(&bad), sizeof(bad));
  assert(g_samples[0].angle == 9054);
}
