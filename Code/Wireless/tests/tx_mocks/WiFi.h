#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <string>
#include <vector>
#include <cassert>
extern uint32_t mockUs, mockMs;
inline uint32_t micros() { return mockUs; }
inline uint32_t millis() { return mockMs; }
inline void advanceUs(uint32_t us) { mockUs += us; mockMs += us / 1000; }
inline void delay(uint32_t ms) { mockMs += ms; mockUs += ms * 1000; }
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
extern int mockLockDepth;
extern bool mockAdvanceAfterUnlock;
inline void enterCritical(portMUX_TYPE*) { assert(mockLockDepth == 0); ++mockLockDepth; }
inline void exitCritical(portMUX_TYPE*) {
  assert(mockLockDepth == 1); --mockLockDepth;
  if (mockAdvanceAfterUnlock) { advanceUs(1000); mockAdvanceAfterUnlock = false; }
}
#define portENTER_CRITICAL(mux) enterCritical(mux)
#define portEXIT_CRITICAL(mux) exitCritical(mux)
struct MockSerial {
  int room = 64;
  std::string output;
  void begin(int baud) { assert(baud == 115200); }
  int availableForWrite() { return room; }
  size_t write(const uint8_t* bytes, size_t count) {
    assert(mockLockDepth == 0 && count <= (size_t)room && count <= 64);
    output.append((const char*)bytes, count);
    return count;
  }
  void printf(const char* format, ...) __attribute__((format(printf, 2, 3))) {
    char text[256]; va_list args; va_start(args, format);
    vsnprintf(text, sizeof(text), format, args); va_end(args);
    output += text;
  }
  void println(const char* text) { output += text; output += '\n'; }
};
extern MockSerial Serial;
struct MockWiFi { void mode(int) {} };
extern MockWiFi WiFi;
#define WIFI_STA 1
