#pragma once
#define WIFI_SECOND_CHAN_NONE 0
#define WIFI_PS_NONE 0
inline int esp_wifi_set_channel(int, int) { return 0; }
inline int esp_wifi_set_ps(int) { return 0; }
