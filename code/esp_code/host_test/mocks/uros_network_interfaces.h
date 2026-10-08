/* Mock of the micro-ROS network interface (host tests). */
#pragma once
#include "esp_err.h"
extern int       mock_netif_init_calls;
extern esp_err_t mock_netif_init_ret;
esp_err_t uros_network_interface_initialize(void);
