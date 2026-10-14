#pragma once
#include <WiFi.h>
#include <esp_idf_version.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_ESPNOW_NO_MEM 12396
enum esp_now_send_status_t { ESP_NOW_SEND_SUCCESS, ESP_NOW_SEND_FAIL };
struct esp_now_send_info_t {};
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,5,0)
using esp_now_send_cb_t = void (*)(const esp_now_send_info_t*, esp_now_send_status_t);
#else
using esp_now_send_cb_t = void (*)(const uint8_t*, esp_now_send_status_t);
#endif
extern esp_now_send_cb_t mockSendCallback;
extern int mockSendResult;
extern uint32_t mockSendDurationUs;
extern std::vector<std::vector<uint8_t>> mockPackets;
struct esp_now_peer_info_t { uint8_t peer_addr[6]; uint8_t channel; bool encrypt; };
inline int esp_now_init() { return ESP_OK; }
inline int esp_now_register_send_cb(esp_now_send_cb_t cb) { mockSendCallback = cb; return ESP_OK; }
inline int esp_now_add_peer(const esp_now_peer_info_t* peer) {
  assert(peer->channel == 6 && !peer->encrypt); return ESP_OK;
}
inline int esp_now_send(const uint8_t*, const uint8_t* data, size_t count) {
  assert(mockLockDepth == 0 && count == 3);
  mockPackets.emplace_back(data, data + count);
  advanceUs(mockSendDurationUs);
  return mockSendResult;
}
