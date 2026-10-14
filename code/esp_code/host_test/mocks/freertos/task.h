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
/* The time of the mock only goes on with the delays: the tick count is the
 * sum of all the delays (mock_delay_total_ms). xTaskDelayUntil() waits with
 * vTaskDelay() until *prev + inc, or returns pdFALSE without waiting if
 * that time already passed (like FreeRTOS). */
TickType_t  xTaskGetTickCount(void);
BaseType_t  xTaskDelayUntil(TickType_t *prev, TickType_t inc);
void        vTaskDelete(TaskHandle_t handle);
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t handle);

/* Run time stats (only used by cpu_stats.c). uxTaskGetSystemState() is
 * not in mock_esp.c: the test of cpu_stats defines it. */
#define configRUN_TIME_COUNTER_TYPE uint32_t
typedef enum { eRunning, eReady, eBlocked, eSuspended, eDeleted } eTaskState;
typedef struct {
    TaskHandle_t                xHandle;
    const char                 *pcTaskName;
    UBaseType_t                 xTaskNumber;
    eTaskState                  eCurrentState;
    UBaseType_t                 uxCurrentPriority;
    UBaseType_t                 uxBasePriority;
    configRUN_TIME_COUNTER_TYPE ulRunTimeCounter;
    StackType_t                *pxStackBase;
    uint32_t                    usStackHighWaterMark;
} TaskStatus_t;
UBaseType_t uxTaskGetSystemState(TaskStatus_t *array, UBaseType_t size,
                                 configRUN_TIME_COUNTER_TYPE *total);

/* Find a created task by name (NULL if it does not exist). */
const mock_task_t *mock_find_task(const char *name);

/* Run a task function (normally an infinite loop) and stop it after
 * max_delays calls to vTaskDelay(), or when it calls vTaskDelete(NULL).
 * Returns the number of vTaskDelay() calls done inside the task. */
int mock_run_task(TaskFunction_t fn, void *arg, int max_delays);

/* Optional hook called on every vTaskDelay() (NULL = none). */
extern void (*mock_delay_hook)(TickType_t ticks);
