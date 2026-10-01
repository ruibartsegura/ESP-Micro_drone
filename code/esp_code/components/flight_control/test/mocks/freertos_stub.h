#pragma once
#include <stdint.h>
typedef unsigned int UBaseType_t;
typedef uint8_t StackType_t;
#define pdMS_TO_TICKS(x) (x)
#define CONFIG_SYSTEM_TASK_STACK 4096
#define CONFIG_SYSTEM_TASK_PRIO  5
static inline void vTaskDelay(int t) { (void)t; }
static inline UBaseType_t uxTaskGetStackHighWaterMark(void *h) { (void)h; return 0; }
static inline int xTaskCreate(void (*f)(void *), const char *n, int s, void *a, int p, void *h) {
    (void)f; (void)n; (void)s; (void)a; (void)p; (void)h; return 1;
}
