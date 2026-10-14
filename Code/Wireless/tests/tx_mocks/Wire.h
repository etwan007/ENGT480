#pragma once
#include <WiFi.h>
struct MockWire {
  uint8_t error = 0;
  uint32_t writeUs = 30, requestUs = 70;
  int count = 2, availableBytes = 2, index = 0, requests = 0;
  uint8_t data[2] = {0x80, 0xFC};
  bool begin() { return true; }
  void setClock(uint32_t hz) { assert(hz == 400000); }
  uint16_t getTimeOut() { return 50; }
  void beginTransmission(int address) { assert(address == 6); }
  void write(int reg) { assert(reg == 3); }
  uint8_t endTransmission(bool stop) { assert(!stop); advanceUs(writeUs); return error; }
  int requestFrom(int address, unsigned countRequested) {
    assert(address == 6 && countRequested == 2); ++requests; index = 0;
    advanceUs(requestUs); return count;
  }
  int available() { return availableBytes; }
  int read() { assert(index < 2); return data[index++]; }
};
extern MockWire Wire;
