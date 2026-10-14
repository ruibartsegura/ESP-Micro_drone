/**
 * Made by Rui B.S.
 * Date: 14/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   CPU usage and real period of the tasks, written in the log to analyse
 *   it later with tools/analyze_cpu.py (nothing is analysed on the drone).
 *   Every CONFIG_CPU_STATS_PERIOD_MS a task writes:
 *     DIAG: CPU,<t_ms>,<window_us>,<task>,<prio>,<run_us>,<stack_free_B>
 *       one line for each FreeRTOS task (also IDLE, WiFi, lwIP...) with the
 *       time it used the CPU in the window (FreeRTOS run time stats, us).
 *     DIAG: PERIOD,<t_ms>,<task>,<target_ms>,<n>,<avg_us>,<max_us>,<late>
 *       one line for each task that calls task_period_tick(): real time
 *       between two iterations. late = iterations of more than 2 x target
 *       (the task lost at least one whole period).
 *   Save the monitor with: idf.py monitor | tee log.txt
 *
 * Functions:
 *   - task_period_tick(): measures the period of the task that calls it.
 *   - period_take(): copies the period of a window and starts a new one.
 *   - run_before(): run time of a task in the previous window.
 *   - cpu_stats_task(): FreeRTOS task that writes the log lines.
 *   - init_cpu_stats(): creates the task.
 */

#include "sdkconfig.h"

#ifdef CONFIG_CPU_STATS_ON
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "cpu_stats.h"

#define CPU_STATS_TASK_STACK 3072
// Over the tasks of the drone: it has to write the log even if they take all
// the CPU. It sleeps almost all the time, so it does not disturb them
#define CPU_STATS_TASK_PRIO 8

#define MAX_TASKS 32   // FreeRTOS tasks (drone + ESP-IDF + WiFi)
#define MAX_PERIODS 12 // Tasks that measure their period

static const char *TAG = "DIAG";
static bool is_init = false;

static task_period_t *periods[MAX_PERIODS];
static int n_periods = 0;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

// Two readings of the run time of the tasks, the window is the difference
static TaskStatus_t prev[MAX_TASKS], cur[MAX_TASKS];


void task_period_tick(task_period_t *p) {
    int64_t now = esp_timer_get_time();

    portENTER_CRITICAL(&lock);
    if (!p->registered) {
        // First call: it goes to the list and starts to count
        if (n_periods < MAX_PERIODS) {
            periods[n_periods++] = p;
        }
        p->registered = true;
        p->t_last_us = now;
        portEXIT_CRITICAL(&lock);
        return;
    }

    int64_t dt = now - p->t_last_us;
    p->t_last_us = now;
    p->n++;
    p->sum_us += dt;
    if (dt > p->max_us) {
        p->max_us = dt;
    }
    if (dt > 2LL * p->period_ms * 1000) {
        p->late++;
    }
    portEXIT_CRITICAL(&lock);
}

static task_period_t period_take(task_period_t *p) {
    portENTER_CRITICAL(&lock);
    task_period_t copy = *p;
    p->n = 0;
    p->late = 0;
    p->sum_us = 0;
    p->max_us = 0;
    portEXIT_CRITICAL(&lock);
    return copy;
}

static configRUN_TIME_COUNTER_TYPE run_before(TaskHandle_t handle,
                                              const TaskStatus_t *list, UBaseType_t n) {
    for (UBaseType_t i = 0; i < n; i++) {
        if (list[i].xHandle == handle) {
            return list[i].ulRunTimeCounter;
        }
    }
    return 0; // New task: all its time is from this window
}

static void cpu_stats_task(void *arg) {
    (void)arg;

    configRUN_TIME_COUNTER_TYPE t_prev, t_cur;
    UBaseType_t n_prev = uxTaskGetSystemState(prev, MAX_TASKS, &t_prev);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_CPU_STATS_PERIOD_MS));

        long long t_ms = esp_timer_get_time() / 1000;
        UBaseType_t n_cur = uxTaskGetSystemState(cur, MAX_TASKS, &t_cur);
        if (n_cur == 0) {
            ESP_LOGW(TAG, "More than %d tasks, raise MAX_TASKS", MAX_TASKS);
            continue;
        }

        // CPU time of each task in the window (unsigned: the counter can wrap)
        unsigned long long window = (configRUN_TIME_COUNTER_TYPE)(t_cur - t_prev);
        for (UBaseType_t i = 0; i < n_cur; i++) {
            configRUN_TIME_COUNTER_TYPE before = run_before(cur[i].xHandle, prev, n_prev);
            unsigned long long run = (configRUN_TIME_COUNTER_TYPE)(cur[i].ulRunTimeCounter
                                                                   - before);
            ESP_LOGI(TAG, "CPU,%lld,%llu,%s,%u,%llu,%u", t_ms, window,
                     cur[i].pcTaskName, (unsigned)cur[i].uxCurrentPriority, run,
                     (unsigned)(cur[i].usStackHighWaterMark * sizeof(StackType_t)));
        }

        // Real period of the tasks that measure it
        for (int k = 0; k < n_periods; k++) {
            task_period_t c = period_take(periods[k]);
            long long avg = c.n ? (long long)(c.sum_us / c.n) : 0;
            ESP_LOGI(TAG, "PERIOD,%lld,%s,%d,%lu,%lld,%lld,%lu", t_ms, c.name,
                     c.period_ms, (unsigned long)c.n, avg, (long long)c.max_us,
                     (unsigned long)c.late);
        }

        memcpy(prev, cur, n_cur * sizeof(TaskStatus_t));
        n_prev = n_cur;
        t_prev = t_cur;
    }
}

void init_cpu_stats(void) {
    if (is_init) {
        return;
    }

    xTaskCreate(cpu_stats_task,
        "cpu_stats",
        CPU_STATS_TASK_STACK,
        NULL,
        CPU_STATS_TASK_PRIO,
        NULL);

    is_init = true;
}

#endif // CONFIG_CPU_STATS_ON
