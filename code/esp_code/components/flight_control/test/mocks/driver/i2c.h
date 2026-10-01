#pragma once
/* En ESP-IDF real esta cabecera arrastra FreeRTOS; attitude_controller.c depende de ello. */
#include "freertos_stub.h"
typedef int i2c_port_t;
#define I2C_NUM_0 0
