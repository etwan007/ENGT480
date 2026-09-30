#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ================= USER SETTINGS =================
#define ESPNOW_CHANNEL        6
#define NUM_SENDERS           3

// USB serial to Pi
#define SERIAL_BAUD           921600

// Control loop rate (handshake rate)
#define CTRL_HZ               100
#define CTRL_PERIOD_MS        (1000UL / CTRL_HZ)

// Consider a sender "present" if we heard within this many ms
#define PRESENT_TIMEOUT_MS    50

// --------- MOTOR PINS (CHANGE THESE) ----------
#define STEP_PIN              18   // step output (LEDC)
#define DIR_PIN               19   // direction output
#define EN_PIN                32   // enable output (ACTIVE LOW like your Arduino)

// Limit switches (ACTIVE LOW)
#define LIM1_PIN              22
#define LIM2_PIN              23

// // --------- STEP PULSE GENERATION ----------
// #define LEDC_CH               0
// #define LEDC_TIMER            0
// #define LEDC_RES_BITS         8    // duty granularity; fine for 50%

// Arduino-equivalent timebase:
// Your old code used 16MHz/64 = 250kHz tick and toggle, so f = 125000/top.
// Keep that same conversion so your Python output "top" still works.
#define TIMER_TICK_HZ         250000.0f
// =================================================

// -------- ESPNOW packet from XIAO --------
typedef struct __attribute__((packed)) {
  uint8_t  sender_id;   // 0..2
  uint16_t angle14;     // 0..16383 or 0xFFFF
} AnglePacket;

// Latest angles (from ESPNOW)
volatile uint16_t g_angle[3] = {0xFFFF, 0xFFFF, 0xFFFF};
volatile uint32_t g_rxMs [3] = {0,0,0};

uint16_t enc[3] = {0xFFFF, 0xFFFF, 0xFFFF};
uint32_t rx[3] = {0, 0, 0};



static inline void setChannel(uint8_t ch) {
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}

// ESPNOW recv callback
void IRAM_ATTR onRecv(const esp_now_recv_info* info, const uint8_t* data, int len) {
  (void)info;
  if (len != (int)sizeof(AnglePacket)) return;

  AnglePacket p;
  memcpy(&p, data, sizeof(p));
  if (p.sender_id >= NUM_SENDERS) return;

  g_angle[p.sender_id] = p.angle14;
  g_rxMs[p.sender_id]  = (uint32_t)millis();
  // Serial.println(g_angle[0]);
}

// ===== Serial helpers =====
// Send int16 big-endian like your Python struct.unpack('>hh')
static inline void writeI16BE(int16_t v) {
  Serial.write((uint8_t)((v >> 8) & 0xFF));
  Serial.write((uint8_t)(v & 0xFF));
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  // ESPNOW init
  WiFi.mode(WIFI_STA);
  setChannel(ESPNOW_CHANNEL);
  esp_wifi_set_ps(WIFI_PS_NONE);

  if (esp_now_init() != ESP_OK) {
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);
}

void loop() {
  static uint32_t lastTick = 0;
  uint32_t now = millis();

  // Run control handshake at fixed rate
  if ((uint32_t)(now - lastTick) >= CTRL_PERIOD_MS) {
    lastTick += CTRL_PERIOD_MS;

    // Snapshot angles; if stale, send 0xFFFF
    for (int i = 0; i < NUM_SENDERS; i++) {
      enc[i] = g_angle[i];
      rx[i]  = g_rxMs[i];
    }

    for (int i = 0; i < NUM_SENDERS; i++) {
      if (rx[i] == 0) {
        enc[i] = 0xFFFF;
      } else {
        uint32_t age = now - rx[i];
        if (age > PRESENT_TIMEOUT_MS) enc[i] = 0xFFFF;
      }
    }

    // ---- SEND 6 BYTES TO PI (big-endian int16): enc0, enc1, enc2
    writeI16BE((int16_t)enc[0]);
    writeI16BE((int16_t)enc[1]);
    writeI16BE((int16_t)enc[2]);
  }
}