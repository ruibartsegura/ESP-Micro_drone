/* Mock of task.h (host tests).
 * xTaskCreate() does not start anything: it saves the task so the test can
 * check its name, stack and priority, and run it with mock_run_task(). */
#pragma once
#include "freertos/FreeRTOS.h"

#define MOCK_MAX_TASKS 16
typedef struct {
    TaskFunction_t fn;
    char           name[32];
    uint32_t       stack;
    UBaseType_t    prio;
    void          *arg;
} mock_task_t;

extern mock_task_t mock_tasks[MOCK_MAX_TASKS];
extern int         mock_task_count;
extern int         mock_delay_calls;     /* number of vTaskDelay() calls      */
extern uint64_t    mock_delay_total_ms;  /* sum of all the delays             */
extern int         mock_task_delete_calls;

BaseType_t  xTaskCreate(TaskFunction_t fn, const char *name, uint32_t stack,
                        void *arg, UBaseType_t prio, TaskHandle_t *handle);
void        vTaskDelay(TickType_t ticks);
void        vTaskDelete(TaskHandle_t handle);
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t handle);

/* Find a created task by name (NULL if it does not exist). */
const mock_task_t *mock_find_task(const char *name);

/* Run a task function (normally an infinite loop) and stop it after
 * max_delays calls to vTaskDelay(), or when it calls vTaskDelete(NULL).
 * Returns the number of vTaskDelay() calls done inside the task. */
int mock_run_task(TaskFunction_t fn, void *arg, int max_delays);

/* Optional hook called on every vTaskDelay() (NULL = none). */
extern void (*mock_delay_hook)(TickType_t ticks);
