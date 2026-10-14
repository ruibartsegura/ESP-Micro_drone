/**
 * Made by Rui B.S.
 * Date: 14/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Public interface of the CPU usage and task period log (cpu_stats.c).
 *   Without CONFIG_CPU_STATS_ON the functions are empty and cost nothing.
 *
 *   Use in the loop of a task:
 *     static task_period_t period = TASK_PERIOD("imu_task", IMU_TASK_PERIOD_MS);
 *     while (1) {
 *         task_period_tick(&period);
 *         ...
 *     }
 *
 * Functions:
 *   - task_period_tick(): measures the real period of the task that calls it.
 *   - init_cpu_stats(): creates the task that writes the log.
 */

#ifndef CPU_STATS_H
#define CPU_STATS_H

#include <stdbool.h>
#include <stdint.h>

#include "sdkconfig.h"

// Real period of one task (filled by task_period_tick())
typedef struct task_period {
    const char *name;   // Name in the log
    int period_ms;      // Period the task wants (its vTaskDelay)
    bool registered;    // Already in the list of the log
    int64_t t_last_us;  // Time of the last iteration
    uint32_t n;         // Iterations in this window
    uint32_t late;      // Iterations longer than 2 x period_ms
    int64_t sum_us;     // Sum of the periods in this window
    int64_t max_us;     // Longest period in this window
} task_period_t;

#define TASK_PERIOD(name_, period_ms_) { .name = (name_), .period_ms = (period_ms_) }

#ifdef CONFIG_CPU_STATS_ON
    void task_period_tick(task_period_t *p);
    void init_cpu_stats(void);
#else
    static inline void task_period_tick(task_period_t *p) { (void)p; }
    static inline void init_cpu_stats(void) {}
#endif

#endif // CPU_STATS_H
