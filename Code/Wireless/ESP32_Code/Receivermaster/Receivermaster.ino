#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "angle_protocol.h"

#define ESPNOW_CHANNEL 6
#define NUM_SENDERS 3
#define SERIAL_BAUD 921600
#define CTRL_HZ 100
#define CTRL_PERIOD_MS (1000UL / CTRL_HZ)

// The radio format stays compatible with TX1/TX2. Only USB serial uses v1.
typedef struct __attribute__((packed)) {
  uint8_t sender_id;
  uint16_t angle14;
} AnglePacket;

static AngleProtocol::Sample g_samples[NUM_SENDERS] = {};
static portMUX_TYPE sampleMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t lastTick = 0;
static uint32_t sequence = 0;

// This callback runs in the Wi-Fi task, not an interrupt handler.
void onRecv(const esp_now_recv_info* info, const uint8_t* data, int len) {
  (void)info;
  if (len != (int)sizeof(AnglePacket)) return;
  AnglePacket packet;
  memcpy(&packet, data, sizeof(packet));
  if (packet.sender_id >= NUM_SENDERS) return;
  if (packet.angle14 >= 16384 && packet.angle14 != 0xFFFF) return;

  portENTER_CRITICAL(&sampleMux);
  g_samples[packet.sender_id].angle = packet.angle14;
  g_samples[packet.sender_id].receivedMs = (uint32_t)millis();
  g_samples[packet.sender_id].seen = true;
  portEXIT_CRITICAL(&sampleMux);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_ps(WIFI_PS_NONE);
  if (esp_now_init() != ESP_OK) {
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);
  lastTick = (uint32_t)millis();
}

void loop() {
  uint32_t scheduledAt = (uint32_t)millis();
  if ((uint32_t)(scheduledAt - lastTick) < CTRL_PERIOD_MS) return;
  // Skip missed slots after a stall instead of emitting a burst of old slots.
  lastTick = scheduledAt;

  AngleProtocol::Sample snapshot[NUM_SENDERS];
  portENTER_CRITICAL(&sampleMux);
  memcpy(snapshot, g_samples, sizeof(snapshot));
  portEXIT_CRITICAL(&sampleMux);
  // Read the clock AFTER the snapshot, so a copied arrival cannot be in the future.
  const uint32_t now = (uint32_t)millis();

  uint8_t frame[AngleProtocol::FRAME_SIZE];
  AngleProtocol::encode(frame, sequence++, now, snapshot);
  // A single write reduces fragmentation. Header + CRC allow reader recovery
  // even if the serial transport still splits, truncates, or loses bytes.
  Serial.write(frame, sizeof(frame));
}
