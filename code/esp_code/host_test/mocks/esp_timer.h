/* Mock of esp_timer.h (host tests). The time is controlled by the test. */
#pragma once
#include <stdint.h>
extern int64_t mock_time_us;        /* value returned by esp_timer_get_time() */
extern int64_t mock_time_step_us;   /* added after every call (0 = frozen clock) */
int64_t esp_timer_get_time(void);
