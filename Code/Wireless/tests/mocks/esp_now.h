#pragma once
#include <stdint.h>
#define ESP_OK 0
struct esp_now_recv_info {};
inline int esp_now_init() { return ESP_OK; }
inline int esp_now_register_recv_cb(void (*)(const esp_now_recv_info*, const uint8_t*, int)) { return ESP_OK; }
