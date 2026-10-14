#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstring>
#include <vector>
#include <cassert>
extern uint32_t mockMillis;
extern int mockLockDepth;
extern void (*mockBeforeLock)();
inline uint32_t millis() { return mockMillis; }
inline void delay(uint32_t ms) { mockMillis += ms; }
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
inline void enterCritical(portMUX_TYPE*) {
  if (mockBeforeLock && mockLockDepth == 0) {
    void (*callback)() = mockBeforeLock;
    mockBeforeLock = nullptr;
    callback();
  }
  ++mockLockDepth;
}
inline void exitCritical(portMUX_TYPE*) { assert(mockLockDepth == 1); --mockLockDepth; }
#define portENTER_CRITICAL(mux) enterCritical(mux)
#define portEXIT_CRITICAL(mux) exitCritical(mux)
struct MockSerial {
  std::vector<std::vector<uint8_t>> frames;
  void begin(int) {}
  size_t write(const uint8_t* bytes, size_t count) {
    assert(mockLockDepth == 0);
    frames.push_back(std::vector<uint8_t>(bytes, bytes + count));
    return count;
  }
};
extern MockSerial Serial;
struct MockWiFi { void mode(int) {} };
extern MockWiFi WiFi;
#define WIFI_STA 1
