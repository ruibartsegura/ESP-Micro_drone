#pragma once
#include "rcl/rcl.h"

/* Mock of rmw_microros/timing.h. It does not count as an rcl call
 * (mock_rcl_fail_at), it only saves the last values. */
typedef int rmw_ret_t;
typedef struct { const rcl_publisher_t *pub; } rmw_publisher_t;

extern int                    mock_uros_pub_timeout_ms;      /* -1 = never called */
extern const rcl_publisher_t *mock_uros_pub_timeout_pub;

rmw_publisher_t *rcl_publisher_get_rmw_handle(const rcl_publisher_t *pub);
rmw_ret_t        rmw_uros_set_publisher_session_timeout(rmw_publisher_t *pub, int session_timeout);
