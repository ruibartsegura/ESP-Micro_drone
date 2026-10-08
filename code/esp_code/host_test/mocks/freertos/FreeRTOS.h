/* Mock of FreeRTOS.h (host tests). */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef int          BaseType_t;
typedef unsigned int UBaseType_t;
typedef uint32_t     TickType_t;
typedef uint8_t      StackType_t;
typedef void *       TaskHandle_t;
typedef void (*TaskFunction_t)(void *);

#define pdTRUE   1
#define pdFALSE  0
#define pdPASS   1
#define pdFAIL   0
#define portTICK_PERIOD_MS 1
#define portMAX_DELAY      0xFFFFFFFFu
#define pdMS_TO_TICKS(ms)  ((TickType_t)(ms))

/* Critical sections: the mock counts how deep we are, so the tests can
 * check that every lock is released (mock_critical_depth == 0) and that a
 * function really used the lock (mock_critical_enter_count). */
typedef struct { int locked; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED { 0 }
extern int mock_critical_depth;
extern int mock_critical_enter_count;
void mock_port_enter_critical(portMUX_TYPE *mux);
void mock_port_exit_critical(portMUX_TYPE *mux);
#define portENTER_CRITICAL(mux) mock_port_enter_critical(mux)
#define portEXIT_CRITICAL(mux)  mock_port_exit_critical(mux)
