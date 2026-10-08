/* Mock of esp_wifi.h (host tests). */
#pragma once
#include "esp_err.h"
typedef enum { WIFI_PS_NONE = 0, WIFI_PS_MIN_MODEM, WIFI_PS_MAX_MODEM } wifi_ps_type_t;
extern int            mock_wifi_set_ps_calls;
extern wifi_ps_type_t mock_wifi_ps_mode;
extern esp_err_t      mock_wifi_set_ps_ret;
esp_err_t esp_wifi_set_ps(wifi_ps_type_t type);
