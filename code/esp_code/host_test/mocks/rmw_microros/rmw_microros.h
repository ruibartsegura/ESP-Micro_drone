#pragma once
#include "rcl/rcl.h"
rcl_ret_t rmw_uros_options_set_udp_address(const char *ip, const char *port, rmw_init_options_t *opts);
