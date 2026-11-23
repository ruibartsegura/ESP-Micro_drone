#pragma once
#include "rcl/rcl.h"
rcl_ret_t rmw_uros_options_set_udp_address(const char *ip, const char *port, rmw_init_options_t *opts);

/* Ping to the agent (pre-flight test). It returns mock_uros_ping_ret. */
#include "rmw_microros/timing.h"
#ifndef RMW_RET_OK
#define RMW_RET_OK 0
#endif
extern rmw_ret_t mock_uros_ping_ret;
extern int mock_uros_ping_calls;        /* number of pings */
extern int mock_uros_ping_timeout_ms;   /* timeout of the last ping */
extern int mock_uros_ping_attempts;     /* attempts of the last ping */
rmw_ret_t rmw_uros_ping_agent(const int timeout_ms, const uint8_t attempts);
