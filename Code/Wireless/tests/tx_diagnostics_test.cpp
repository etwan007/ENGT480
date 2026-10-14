// Exercises the actual TX1 sketch; no board or connected motor is required.
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
uint32_t mockUs = 0, mockMs = 0;
int mockLockDepth = 0;
bool mockAdvanceAfterUnlock = false;
MockSerial Serial;
MockWiFi WiFi;
MockWire Wire;
esp_now_send_cb_t mockSendCallback = nullptr;
int mockSendResult = ESP_OK;
uint32_t mockSendDurationUs = 0;
std::vector<std::vector<uint8_t>> mockPackets;
#include "../Transmitters/TX1/TX1.ino"

static void reset(uint32_t us = 0, uint32_t ms = 0) {
  mockUs = us; mockMs = ms; mockLockDepth = 0; mockAdvanceAfterUnlock = false;
  Serial = {}; Wire = {}; diag = {}; delivery = {};
  mockPackets.clear(); mockSendResult = ESP_OK; mockSendDurationUs = 0;
  logLength = logOffset = 0; nextSendUs = us;
  lastReportMs = lastDeferredMs = ms;
}
static uint16_t rawPacket() {
  uint16_t raw; memcpy(&raw, mockPackets.back().data() + 1, 2); return raw;
}
static std::string summary() {
  mockMs = lastReportMs + 1000;
  serviceDiagnostics();
  assert(logLength > 0 && logLength < sizeof(logBuffer));
  return std::string(logBuffer, logLength);
}
static void has(const std::string& text, const char* field) {
  assert(text.find(field) != std::string::npos);
}

int main() {
  reset(); setup();
  assert(mockSendCallback == onSent);
  has(Serial.output, "i2c_timeout_ms=50");
  loop();
  assert(mockPackets.back()[0] == 0 && rawPacket() == 8255);
  assert(diag.reads == 1 && diag.queued == 1 && diag.readMaxUs == 100);
  assert(diag.writeMaxUs == 30 && diag.requestMaxUs == 70);
  std::string before = Serial.output;
  mockSendCallback(nullptr, ESP_NOW_SEND_SUCCESS);
  mockSendCallback(nullptr, ESP_NOW_SEND_FAIL);
  assert(Serial.output == before && mockLockDepth == 0);
  auto line = summary();
  has(line, "delivery_ok=1 delivery_fail=1");
  has(line, "read_max_us=100");
  assert(diag.readMaxUs == 0 && diag.reads == 1);

  // A 50 ms register-write timeout delays the next send and preserves 0xFFFF.
  reset(); loop(); mockUs = 5000; mockMs = 5;
  Wire.error = 5; Wire.writeUs = 50000;
  loop();
  assert(Wire.requests == 1 && rawPacket() == 0xFFFF);
  assert(diag.readFail == 1 && diag.writeFail == 1 && diag.shortRead == 0);
  assert(diag.slowRead == 1 && diag.gapMaxUs == 54900 && diag.gaps50ms == 1);
  line = summary();
  has(line, "read_max_us=50000 read_max_ms=55 write_max_us=50000");
  has(line, "last_read_fail_ms=55 last_i2c_error=5 last_read_bytes=-1");
  has(line, "gap_max_us=54900 gap_max_ms=55 gaps_50ms=1");

  // A request timeout, a short response, and a successful but slow read.
  reset(); Wire.requestUs = 50000; Wire.count = Wire.availableBytes = 0; loop();
  assert(diag.shortRead == 1 && diag.writeFail == 0 && rawPacket() == 0xFFFF);
  line = summary(); has(line, "request_max_us=50000"); has(line, "last_read_bytes=0");
  reset(); Wire.count = 1; Wire.availableBytes = 1; loop();
  assert(diag.readFail == 1 && Wire.index == 0);
  reset(); Wire.requestUs = 50000; loop();
  assert(diag.slowRead == 1 && diag.readFail == 0 && rawPacket() == 8255);

  // Send-call rejection is distinct from asynchronous delivery failure.
  reset(); mockSendResult = ESP_ERR_ESPNOW_NO_MEM; mockSendDurationUs = 1000; loop();
  assert(diag.attempts == 1 && diag.queued == 0 && diag.sendError == 1);
  line = summary(); has(line, "last_send_error=12396"); has(line, "send_max_us=1000");
  has(line, "delivery_ok=0 delivery_fail=0 delivery_ok_age_us=4294967295");

  // Catch-up schedule stays unchanged and tracks lateness separately.
  reset(); loop(); mockUs = 60000; mockMs = 60; loop();
  assert(diag.lateMaxUs == 55000 && diag.gapMaxUs == 60000);
  assert(nextSendUs == 10000);

  // No available serial space: sends proceed and maxima survive deferred logs.
  reset(); Serial.room = 0; loop(); line = summary();
  mockMs += 1000; mockUs += 1000000; loop();
  assert(Serial.output.empty() && logOffset == 0 && diag.attempts == 2);
  assert(diag.logDeferred == 1 && diag.gapMaxUs == 1000100);
  serviceDiagnostics(); assert(diag.logDeferred == 1);
  Serial.room = 7;
  while (logOffset < logLength) serviceDiagnostics();
  // Draining the old line can queue the next summary immediately.
  assert(Serial.output.find("TX1_DIAG") != std::string::npos);
  assert(logLength < sizeof(logBuffer));

  // Callback snapshot's clock comes after unlocking, and micros wraps safely.
  reset(0xFFFFFF00u, 100); onSent(nullptr, ESP_NOW_SEND_SUCCESS);
  mockUs = 0x100; mockMs = 1100; mockAdvanceAfterUnlock = true;
  serviceDiagnostics(); line.assign(logBuffer, logLength);
  has(line, "delivery_ok_age_us=1512");
  reset(0xFFFFFF00u, 0xFFFFFF00u); loop();
  mockUs = 0xFFFFFF00u + 5000u; mockMs += 5; loop();
  assert(diag.gapMaxUs == 5000 && diag.gaps50ms == 0);
  mockMs += 1000; serviceDiagnostics(); assert(logLength > 0);

  // Ensure a maximum-width summary fits, terminates, and remains complete.
  reset();
  diag.reads = diag.readFail = diag.writeFail = diag.shortRead = diag.slowRead = UINT32_MAX;
  diag.readMaxUs = diag.readMaxMs = diag.writeMaxUs = diag.requestMaxUs = UINT32_MAX;
  diag.lastReadFailMs = UINT32_MAX; diag.lastI2cError = 255; diag.lastReadBytes = -2147483647;
  diag.attempts = diag.queued = diag.sendError = diag.lastSendErrorMs = UINT32_MAX;
  diag.lastSendError = -2147483647; diag.sendMaxUs = UINT32_MAX;
  diag.gapMaxUs = diag.gapMaxMs = diag.gaps50ms = diag.lateMaxUs = diag.logDeferred = UINT32_MAX;
  delivery.ok = delivery.fail = delivery.lastFailMs = UINT32_MAX;
  line = summary(); assert(line.back() == '\n'); has(line, "log_deferred=4294967295\n");
  std::printf("TX1 diagnostics checks passed (IDF minor %d, maximum line %zu bytes).\n", MOCK_IDF_MINOR, line.size());
}
