/**
 * Mock of rcl/rcl.h + rclc (host tests).
 *
 * Every init/add call is saved in mock_ros_entities / mock_exec_handles,
 * so a test can check the topic names, types, QoS and the executor
 * handles. Any call can be forced to fail with mock_rcl_fail_at. The
 * message type support is the type name as a string ("sensor_msgs/msg/Imu").
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "rmw/qos_profiles.h"
#include "mock_ros_msgs.h"

typedef int rcl_ret_t;
#define RCL_RET_OK      0
#define RCL_RET_ERROR   1
#define RCL_RET_TIMEOUT 2

#define RCL_MS_TO_NS(ms) ((int64_t)(ms) * 1000000LL)
#define RCLC_UNUSED(x)   (void)(x)

typedef char rosidl_message_type_support_t;
typedef char rosidl_service_type_support_t;
#define ROSIDL_GET_MSG_TYPE_SUPPORT(pkg, sub, type) ((const rosidl_message_type_support_t *)(#pkg "/" #sub "/" #type))
#define ROSIDL_GET_SRV_TYPE_SUPPORT(pkg, sub, type) ((const rosidl_service_type_support_t *)(#pkg "/" #sub "/" #type))

typedef struct { int unused; } rcl_allocator_t;
typedef struct { int unused; } rmw_init_options_t;
typedef struct { rmw_init_options_t rmw; int initialised; } rcl_init_options_t;
typedef struct { int id; } rcl_context_t;
typedef struct { int id; } rcl_node_t;
typedef struct { int id; } rcl_publisher_t;
typedef struct { int id; } rcl_subscription_t;
typedef struct { int id; } rcl_service_t;
typedef struct rcl_timer_s { int id; } rcl_timer_t;
typedef void (*rcl_timer_callback_t)(rcl_timer_t *, int64_t);

rcl_allocator_t     rcl_get_default_allocator(void);
rcl_init_options_t  rcl_get_zero_initialized_init_options(void);
rcl_ret_t           rcl_init_options_init(rcl_init_options_t *opts, rcl_allocator_t alloc);
rmw_init_options_t *rcl_init_options_get_rmw_init_options(rcl_init_options_t *opts);
rcl_ret_t           rcl_publish(const rcl_publisher_t *pub, const void *msg, void *alloc);
bool                rcl_service_is_valid(const rcl_service_t *srv);

/* ---- recorder ---- */
typedef enum { MOCK_ROS_NODE, MOCK_ROS_PUB, MOCK_ROS_SUB, MOCK_ROS_SRV, MOCK_ROS_TIMER } mock_ros_kind_t;
typedef struct {
    mock_ros_kind_t      kind;
    const void          *handle;       /* &publisher, &subscription, ... */
    char                 name[64];     /* topic / service / node name    */
    char                 type[64];     /* "pkg/msg/Type"                 */
    bool                 has_qos;
    rmw_qos_profile_t    qos;
    bool                 best_effort;  /* *_init_best_effort was used    */
    int64_t              period_ns;    /* timers                         */
    rcl_timer_callback_t timer_cb;     /* timers                         */
} mock_ros_entity_t;

#define MOCK_ROS_MAX 32
extern mock_ros_entity_t mock_ros_entities[MOCK_ROS_MAX];
extern int               mock_ros_entity_count;
extern int               mock_rcl_calls;        /* calls to any mocked rcl/rclc function */
extern int               mock_rcl_fail_at;      /* make call number N fail (1..), 0 = never */
extern int               mock_rcl_publish_calls;
extern const void       *mock_rcl_last_pub;
extern const void       *mock_rcl_last_msg;
extern char              mock_uros_agent_ip[64];
extern char              mock_uros_agent_port[16];

const mock_ros_entity_t *mock_ros_find(mock_ros_kind_t kind, const char *name);
int                      mock_ros_count(mock_ros_kind_t kind);
rcl_ret_t                mock_rcl_next(void);   /* used by the mocks: count + failure injection */
