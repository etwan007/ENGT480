// TX1: unchanged three-byte angle packets, with local USB diagnostics.
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <Wire.h>
#include <stdio.h>

#define ESPNOW_CHANNEL 6
#define SEND_HZ 200
#define SEND_PERIOD_US (1000000UL / SEND_HZ)
#define MT6701_ADDR 0x06
#define MT6701_ANGLE_MSB 0x03
#define SENDER_ID 0
#define DIAG_PERIOD_MS 1000UL

uint8_t MASTER_MAC[6] = {0xFC, 0xB4, 0x67, 0xF6, 0x50, 0xF4};
typedef struct __attribute__((packed)) {
  uint8_t sender_id;
  uint16_t raw14; // 0..16383, or 0xFFFF if encoder read failed
} AnglePacket;
static_assert(sizeof(AnglePacket) == 3, "Receiver requires three-byte packets");

// Only the loop touches encoder/send-call statistics. Counters are cumulative;
// maxima describe the interval since the last successfully queued summary.
static struct {
  uint32_t reads = 0, readFail = 0, writeFail = 0, shortRead = 0, slowRead = 0;
  uint32_t readMaxUs = 0, readMaxMs = 0, writeMaxUs = 0, requestMaxUs = 0;
  uint32_t lastReadFailMs = 0;
  uint8_t lastI2cError = 0;
  int lastReadBytes = -1; // -1 means requestFrom was not reached
  uint32_t attempts = 0, queued = 0, sendError = 0;
  esp_err_t lastSendError = ESP_OK;
  uint32_t lastSendErrorMs = 0, sendMaxUs = 0;
  uint32_t gapMaxUs = 0, gapMaxMs = 0, gaps50ms = 0, lateMaxUs = 0;
  uint32_t lastAttemptUs = 0;
  bool attempted = false;
  uint32_t logDeferred = 0;
} diag;

// Callback state is protected because the Wi-Fi task runs independently.
static portMUX_TYPE deliveryMux = portMUX_INITIALIZER_UNLOCKED;
static struct {
  uint32_t ok = 0, fail = 0, lastOkUs = 0, lastFailMs = 0;
  bool seenOk = false;
} delivery;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void onSent(const esp_now_send_info_t*, esp_now_send_status_t status) {
#else
static void onSent(const uint8_t*, esp_now_send_status_t status) {
#endif
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();
  portENTER_CRITICAL(&deliveryMux);
  if (status == ESP_NOW_SEND_SUCCESS) {
    ++delivery.ok;
    delivery.lastOkUs = nowUs;
    delivery.seenOk = true;
  } else {
    ++delivery.fail;
    delivery.lastFailMs = nowMs;
  }
  portEXIT_CRITICAL(&deliveryMux);
  // Never print or perform I2C work in the high-priority Wi-Fi callback.
}

static bool mt6701_readAngle14(uint16_t &angle14) {
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(MT6701_ANGLE_MSB);
  uint32_t start = micros();
  uint8_t error = Wire.endTransmission(false);
  uint32_t duration = micros() - start;
  if (duration > diag.writeMaxUs) diag.writeMaxUs = duration;
  if (error != 0) {
    ++diag.writeFail;
    diag.lastI2cError = error;
    diag.lastReadBytes = -1;
    return false;
  }
  start = micros();
  int n = Wire.requestFrom(MT6701_ADDR, 2u);
  duration = micros() - start;
  if (duration > diag.requestMaxUs) diag.requestMaxUs = duration;
  if (n != 2 || Wire.available() < 2) {
    ++diag.shortRead;
    diag.lastI2cError = 0;
    diag.lastReadBytes = n;
    return false;
  }
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  angle14 = (((uint16_t)msb << 6) | (lsb >> 2)) & 0x3FFF;
  return true;
}

static uint32_t nextSendUs = 0, lastReportMs = 0, lastDeferredMs = 0;
static char logBuffer[1024];
static size_t logLength = 0, logOffset = 0;

static void serviceDiagnostics() {
  // Avoid waiting for USB/UART space. There is only one serial writer.
  int room = Serial.availableForWrite();
  if (room > 0 && logOffset < logLength) {
    size_t count = logLength - logOffset;
    if (count > (size_t)room) count = (size_t)room;
    if (count > 64) count = 64;
    logOffset += Serial.write((const uint8_t*)logBuffer + logOffset, count);
  }
  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastReportMs) < DIAG_PERIOD_MS) return;
  uint32_t windowMs = nowMs - lastReportMs;
  if (logOffset < logLength) {
    // Retain counters/maxima; do not block transmission for a slow monitor.
    if ((uint32_t)(nowMs - lastDeferredMs) >= DIAG_PERIOD_MS) {
      ++diag.logDeferred;
      lastDeferredMs = nowMs;
    }
    return;
  }
  uint32_t ok, fail, lastOkUs, lastFailMs;
  bool seenOk;
  portENTER_CRITICAL(&deliveryMux);
  ok = delivery.ok;
  fail = delivery.fail;
  lastOkUs = delivery.lastOkUs;
  lastFailMs = delivery.lastFailMs;
  seenOk = delivery.seenOk;
  portEXIT_CRITICAL(&deliveryMux);
  // Capture the clock AFTER the callback snapshot to avoid age underflow.
  uint32_t okAgeUs = seenOk ? micros() - lastOkUs : UINT32_MAX;
  int length = snprintf(logBuffer, sizeof(logBuffer),
    "TX1_DIAG ms=%lu window_ms=%lu reads=%lu read_fail=%lu write_fail=%lu short_read=%lu slow_read=%lu "
    "read_max_us=%lu read_max_ms=%lu write_max_us=%lu request_max_us=%lu "
    "last_read_fail_ms=%lu last_i2c_error=%u last_read_bytes=%d "
    "attempts=%lu queued=%lu send_error=%lu last_send_error=%d last_send_error_ms=%lu send_max_us=%lu "
    "gap_max_us=%lu gap_max_ms=%lu gaps_50ms=%lu late_max_us=%lu "
    "delivery_ok=%lu delivery_fail=%lu delivery_ok_age_us=%lu last_delivery_fail_ms=%lu log_deferred=%lu\n",
    (unsigned long)nowMs, (unsigned long)windowMs,
    (unsigned long)diag.reads, (unsigned long)diag.readFail,
    (unsigned long)diag.writeFail, (unsigned long)diag.shortRead, (unsigned long)diag.slowRead,
    (unsigned long)diag.readMaxUs, (unsigned long)diag.readMaxMs,
    (unsigned long)diag.writeMaxUs, (unsigned long)diag.requestMaxUs,
    (unsigned long)diag.lastReadFailMs, (unsigned)diag.lastI2cError, diag.lastReadBytes,
    (unsigned long)diag.attempts, (unsigned long)diag.queued, (unsigned long)diag.sendError,
    (int)diag.lastSendError, (unsigned long)diag.lastSendErrorMs, (unsigned long)diag.sendMaxUs,
    (unsigned long)diag.gapMaxUs, (unsigned long)diag.gapMaxMs,
    (unsigned long)diag.gaps50ms, (unsigned long)diag.lateMaxUs,
    (unsigned long)ok, (unsigned long)fail, (unsigned long)okAgeUs,
    (unsigned long)lastFailMs, (unsigned long)diag.logDeferred);
  if (length < 0 || (size_t)length >= sizeof(logBuffer)) {
    logLength = logOffset = 0;
    ++diag.logDeferred;
    return; // Keep maxima if formatting could not produce a complete line.
  }
  logLength = (size_t)length;
  logOffset = 0;
  lastReportMs = nowMs;
  diag.readMaxUs = diag.readMaxMs = diag.writeMaxUs = diag.requestMaxUs = 0;
  diag.sendMaxUs = diag.gapMaxUs = diag.gapMaxMs = diag.lateMaxUs = 0;
}

static void requireOk(const char* operation, esp_err_t error) {
  if (error == ESP_OK) return;
  Serial.printf("TX1_SETUP_ERROR operation=%s error=%d\n", operation, (int)error);
  while (true) delay(1000);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  if (!Wire.begin()) {
    Serial.println("TX1_SETUP_ERROR operation=Wire.begin");
    while (true) delay(1000);
  }
  Wire.setClock(400000);
  // Observe the existing timeout; do not change it while diagnosing the gaps.
  Serial.printf("TX1_CONFIG send_hz=%u channel=%u i2c_timeout_ms=%u idf=%u.%u.%u\n",
                (unsigned)SEND_HZ, (unsigned)ESPNOW_CHANNEL, (unsigned)Wire.getTimeOut(),
                (unsigned)ESP_IDF_VERSION_MAJOR, (unsigned)ESP_IDF_VERSION_MINOR,
                (unsigned)ESP_IDF_VERSION_PATCH);
  WiFi.mode(WIFI_STA);
  requireOk("set_channel", esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
  requireOk("set_ps", esp_wifi_set_ps(WIFI_PS_NONE));
  requireOk("esp_now_init", esp_now_init());
  requireOk("register_send_cb", esp_now_register_send_cb(onSent));
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, MASTER_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  requireOk("add_peer", esp_now_add_peer(&peer));
  nextSendUs = micros();
  lastReportMs = millis();
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextSendUs) >= 0) {
    uint32_t lateUs = now - nextSendUs;
    if (lateUs > diag.lateMaxUs) diag.lateMaxUs = lateUs;
    // Preserve the original scheduler, including its catch-up after a stall.
    nextSendUs += SEND_PERIOD_US;
    uint16_t angle14 = 0;
    uint32_t start = micros();
    bool valid = mt6701_readAngle14(angle14);
    uint32_t readUs = micros() - start;
    ++diag.reads;
    if (readUs >= SEND_PERIOD_US) ++diag.slowRead;
    if (readUs > diag.readMaxUs) {
      diag.readMaxUs = readUs;
      diag.readMaxMs = millis();
    }
    if (!valid) {
      ++diag.readFail;
      diag.lastReadFailMs = millis();
    }
    AnglePacket packet = {(uint8_t)SENDER_ID, valid ? angle14 : (uint16_t)0xFFFF};
    uint32_t sendUs = micros();
    if (diag.attempted) {
      uint32_t gapUs = sendUs - diag.lastAttemptUs;
      if (gapUs >= 50000UL) ++diag.gaps50ms;
      if (gapUs > diag.gapMaxUs) {
        diag.gapMaxUs = gapUs;
        diag.gapMaxMs = millis();
      }
    }
    diag.attempted = true;
    diag.lastAttemptUs = sendUs;
    ++diag.attempts;
    esp_err_t result = esp_now_send(MASTER_MAC, (uint8_t*)&packet, sizeof(packet));
    uint32_t callUs = micros() - sendUs;
    if (callUs > diag.sendMaxUs) diag.sendMaxUs = callUs;
    if (result == ESP_OK) {
      ++diag.queued;
    } else {
      ++diag.sendError;
      diag.lastSendError = result;
      diag.lastSendErrorMs = millis();
    }
  }
  serviceDiagnostics();
}
