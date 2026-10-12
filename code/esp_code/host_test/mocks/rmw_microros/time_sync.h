#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "rmw_microros/timing.h"

/* Mock of rmw_microros/time_sync.h. It does not count as an rcl call
 * (mock_rcl_fail_at). rmw_uros_sync_session() marks the clock as
 * synchronized when mock_uros_sync_ret is 0. */
extern bool      mock_uros_synced;      /* rmw_uros_epoch_synchronized() */
extern int64_t   mock_uros_epoch_ns;    /* rmw_uros_epoch_nanos() */
extern int       mock_uros_sync_calls;
extern int       mock_uros_sync_timeout_ms;  /* last timeout, -1 = never called */
extern rmw_ret_t mock_uros_sync_ret;

bool      rmw_uros_epoch_synchronized(void);
int64_t   rmw_uros_epoch_millis(void);
int64_t   rmw_uros_epoch_nanos(void);
rmw_ret_t rmw_uros_sync_session(const int timeout_ms);
