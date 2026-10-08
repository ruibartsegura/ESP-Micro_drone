/* Mock of rclc/rclc.h (host tests). See rcl/rcl.h. */
#pragma once
#include "rcl/rcl.h"

typedef struct { rcl_context_t context; int initialised; } rclc_support_t;

rcl_ret_t rclc_support_init_with_options(rclc_support_t *support, int argc, char const *const *argv,
                                         rcl_init_options_t *opts, rcl_allocator_t *alloc);
rcl_ret_t rclc_node_init_default(rcl_node_t *node, const char *name, const char *ns, rclc_support_t *support);
rcl_ret_t rclc_publisher_init_default(rcl_publisher_t *pub, const rcl_node_t *node,
                                      const rosidl_message_type_support_t *type, const char *topic);
rcl_ret_t rclc_publisher_init_best_effort(rcl_publisher_t *pub, const rcl_node_t *node,
                                          const rosidl_message_type_support_t *type, const char *topic);
rcl_ret_t rclc_publisher_init(rcl_publisher_t *pub, const rcl_node_t *node,
                              const rosidl_message_type_support_t *type, const char *topic,
                              const rmw_qos_profile_t *qos);
rcl_ret_t rclc_subscription_init_default(rcl_subscription_t *sub, const rcl_node_t *node,
                                         const rosidl_message_type_support_t *type, const char *topic);
rcl_ret_t rclc_subscription_init_best_effort(rcl_subscription_t *sub, const rcl_node_t *node,
                                             const rosidl_message_type_support_t *type, const char *topic);
rcl_ret_t rclc_subscription_init(rcl_subscription_t *sub, const rcl_node_t *node,
                                 const rosidl_message_type_support_t *type, const char *topic,
                                 const rmw_qos_profile_t *qos);
rcl_ret_t rclc_service_init_default(rcl_service_t *srv, const rcl_node_t *node,
                                    const rosidl_service_type_support_t *type, const char *name);
rcl_ret_t rclc_timer_init_default2(rcl_timer_t *timer, rclc_support_t *support, int64_t period_ns,
                                   rcl_timer_callback_t cb, bool autostart);
