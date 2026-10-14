#pragma once
#include <esp_now.h>
#define WIFI_SECOND_CHAN_NONE 0
#define WIFI_PS_NONE 0
inline int esp_wifi_set_channel(uint8_t ch, int) { assert(ch == 6); return ESP_OK; }
inline int esp_wifi_set_ps(int) { return ESP_OK; }
