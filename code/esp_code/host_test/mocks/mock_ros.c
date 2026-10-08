/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Implementation of the micro-ROS mocks for the host tests (rcl, rclc,
 *   executor, rmw_microros and the message helpers). Every init call is
 *   saved as an "entity" and every executor add as a "handle", so the
 *   tests can check names, types, QoS and the number of handles. Any call
 *   can be forced to fail with mock_rcl_fail_at.
 *
 * Functions:
 *   - mock_reset_ros(): resets the micro-ROS mocks.
 *   - mock_ros_find() / mock_ros_count(): look for saved entities.
 *   - mock_rcl_next(): counts a call and applies the failure injection.
 *   - The rest are the mocked rcl / rclc / rosidl functions.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mock_reset.h"
#include "rcl/rcl.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rmw_microros/rmw_microros.h"
#include "mock_ros_msgs.h"

void mock_task_exit(void);   /* mock_esp.c */

mock_ros_entity_t  mock_ros_entities[MOCK_ROS_MAX];
int                mock_ros_entity_count;
int                mock_rcl_calls;
int                mock_rcl_fail_at;
int                mock_rcl_publish_calls;
const void        *mock_rcl_last_pub;
const void        *mock_rcl_last_msg;
char               mock_uros_agent_ip[64];
char               mock_uros_agent_port[16];

mock_exec_handle_t mock_exec_handles[MOCK_ROS_MAX];
int                mock_exec_handle_count;
size_t             mock_exec_max_handles;
int                mock_spin_calls;
int                mock_spin_max;
rcl_ret_t          mock_spin_ret;

rcl_ret_t mock_rcl_next(void) {
    mock_rcl_calls++;
    if (mock_rcl_fail_at && mock_rcl_calls == mock_rcl_fail_at) return RCL_RET_ERROR;
    return RCL_RET_OK;
}

static mock_ros_entity_t *add_entity(mock_ros_kind_t kind, const void *handle,
                                     const char *name, const char *type) {
    if (mock_ros_entity_count >= MOCK_ROS_MAX) return NULL;
    mock_ros_entity_t *e = &mock_ros_entities[mock_ros_entity_count++];
    memset(e, 0, sizeof *e);
    e->kind = kind;
    e->handle = handle;
    snprintf(e->name, sizeof e->name, "%s", name ? name : "");
    snprintf(e->type, sizeof e->type, "%s", type ? type : "");
    return e;
}

const mock_ros_entity_t *mock_ros_find(mock_ros_kind_t kind, const char *name) {
    for (int i = 0; i < mock_ros_entity_count; i++)
        if (mock_ros_entities[i].kind == kind && strcmp(mock_ros_entities[i].name, name) == 0)
            return &mock_ros_entities[i];
    return NULL;
}

int mock_ros_count(mock_ros_kind_t kind) {
    int n = 0;
    for (int i = 0; i < mock_ros_entity_count; i++)
        if (mock_ros_entities[i].kind == kind) n++;
    return n;
}

/* ---- rcl ---- */
rcl_allocator_t rcl_get_default_allocator(void) { return (rcl_allocator_t){ 0 }; }
rcl_init_options_t rcl_get_zero_initialized_init_options(void) { return (rcl_init_options_t){ 0 }; }

rcl_ret_t rcl_init_options_init(rcl_init_options_t *opts, rcl_allocator_t alloc) {
    (void)alloc;
    rcl_ret_t r = mock_rcl_next();
    if (r == RCL_RET_OK) opts->initialised = 1;
    return r;
}

rmw_init_options_t *rcl_init_options_get_rmw_init_options(rcl_init_options_t *opts) { return &opts->rmw; }

rcl_ret_t rcl_publish(const rcl_publisher_t *pub, const void *msg, void *alloc) {
    (void)alloc;
    mock_rcl_publish_calls++;
    mock_rcl_last_pub = pub;
    mock_rcl_last_msg = msg;
    return RCL_RET_OK;
}

bool rcl_service_is_valid(const rcl_service_t *srv) { return srv != NULL; }

rcl_ret_t rmw_uros_options_set_udp_address(const char *ip, const char *port, rmw_init_options_t *opts) {
    (void)opts;
    snprintf(mock_uros_agent_ip, sizeof mock_uros_agent_ip, "%s", ip);
    snprintf(mock_uros_agent_port, sizeof mock_uros_agent_port, "%s", port);
    return mock_rcl_next();
}

/* ---- rclc ---- */
rcl_ret_t rclc_support_init_with_options(rclc_support_t *support, int argc, char const *const *argv,
                                         rcl_init_options_t *opts, rcl_allocator_t *alloc) {
    (void)argc; (void)argv; (void)opts; (void)alloc;
    rcl_ret_t r = mock_rcl_next();
    if (r == RCL_RET_OK) support->initialised = 1;
    return r;
}

rcl_ret_t rclc_node_init_default(rcl_node_t *node, const char *name, const char *ns, rclc_support_t *support) {
    (void)ns; (void)support;
    rcl_ret_t r = mock_rcl_next();
    if (r == RCL_RET_OK) add_entity(MOCK_ROS_NODE, node, name, "");
    return r;
}

static rcl_ret_t add_endpoint(mock_ros_kind_t kind, const void *h, const char *type, const char *topic,
                              const rmw_qos_profile_t *qos, bool best_effort) {
    rcl_ret_t r = mock_rcl_next();
    if (r != RCL_RET_OK) return r;
    mock_ros_entity_t *e = add_entity(kind, h, topic, type);
    if (e) {
        e->best_effort = best_effort;
        if (qos) { e->has_qos = true; e->qos = *qos; }
    }
    return r;
}

rcl_ret_t rclc_publisher_init_default(rcl_publisher_t *pub, const rcl_node_t *node,
                                      const rosidl_message_type_support_t *type, const char *topic) {
    (void)node; return add_endpoint(MOCK_ROS_PUB, pub, type, topic, NULL, false);
}
rcl_ret_t rclc_publisher_init_best_effort(rcl_publisher_t *pub, const rcl_node_t *node,
                                          const rosidl_message_type_support_t *type, const char *topic) {
    (void)node; return add_endpoint(MOCK_ROS_PUB, pub, type, topic, NULL, true);
}
rcl_ret_t rclc_publisher_init(rcl_publisher_t *pub, const rcl_node_t *node,
                              const rosidl_message_type_support_t *type, const char *topic,
                              const rmw_qos_profile_t *qos) {
    (void)node; return add_endpoint(MOCK_ROS_PUB, pub, type, topic, qos, false);
}
rcl_ret_t rclc_subscription_init_default(rcl_subscription_t *sub, const rcl_node_t *node,
                                         const rosidl_message_type_support_t *type, const char *topic) {
    (void)node; return add_endpoint(MOCK_ROS_SUB, sub, type, topic, NULL, false);
}
rcl_ret_t rclc_subscription_init_best_effort(rcl_subscription_t *sub, const rcl_node_t *node,
                                             const rosidl_message_type_support_t *type, const char *topic) {
    (void)node; return add_endpoint(MOCK_ROS_SUB, sub, type, topic, NULL, true);
}
rcl_ret_t rclc_subscription_init(rcl_subscription_t *sub, const rcl_node_t *node,
                                 const rosidl_message_type_support_t *type, const char *topic,
                                 const rmw_qos_profile_t *qos) {
    (void)node; return add_endpoint(MOCK_ROS_SUB, sub, type, topic, qos, false);
}
rcl_ret_t rclc_service_init_default(rcl_service_t *srv, const rcl_node_t *node,
                                    const rosidl_service_type_support_t *type, const char *name) {
    (void)node; return add_endpoint(MOCK_ROS_SRV, srv, type, name, NULL, false);
}

rcl_ret_t rclc_timer_init_default2(rcl_timer_t *timer, rclc_support_t *support, int64_t period_ns,
                                   rcl_timer_callback_t cb, bool autostart) {
    (void)support; (void)autostart;
    rcl_ret_t r = mock_rcl_next();
    if (r != RCL_RET_OK) return r;
    mock_ros_entity_t *e = add_entity(MOCK_ROS_TIMER, timer, "", "");
    if (e) { e->period_ns = period_ns; e->timer_cb = cb; }
    return r;
}

/* ---- executor ---- */
rcl_ret_t rclc_executor_init(rclc_executor_t *exec, rcl_context_t *ctx, size_t handles, const rcl_allocator_t *alloc) {
    (void)ctx; (void)alloc;
    rcl_ret_t r = mock_rcl_next();
    if (r != RCL_RET_OK) return r;
    exec->max_handles = handles;
    exec->used = 0;
    exec->initialised = 1;
    mock_exec_max_handles = handles;
    return r;
}

/* Like the real executor, adding more handles than reserved fails. */
static rcl_ret_t add_handle(rclc_executor_t *exec, mock_ros_kind_t kind, const void *entity,
                            void *msg, void *res, void *cb, rclc_executor_handle_invocation_t inv) {
    rcl_ret_t r = mock_rcl_next();
    if (r != RCL_RET_OK) return r;
    if (!exec->initialised || exec->used >= exec->max_handles) return RCL_RET_ERROR;
    exec->used++;
    if (mock_exec_handle_count < MOCK_ROS_MAX)
        mock_exec_handles[mock_exec_handle_count++] = (mock_exec_handle_t){ kind, entity, msg, res, cb, inv };
    return RCL_RET_OK;
}

rcl_ret_t rclc_executor_add_timer(rclc_executor_t *exec, rcl_timer_t *timer) {
    return add_handle(exec, MOCK_ROS_TIMER, timer, NULL, NULL, NULL, ALWAYS);
}
rcl_ret_t rclc_executor_add_subscription(rclc_executor_t *exec, rcl_subscription_t *sub, void *msg,
                                         rclc_subscription_callback_t cb,
                                         rclc_executor_handle_invocation_t inv) {
    return add_handle(exec, MOCK_ROS_SUB, sub, msg, NULL, (void *)cb, inv);
}
rcl_ret_t rclc_executor_add_service(rclc_executor_t *exec, rcl_service_t *srv, void *req, void *res,
                                    rclc_service_callback_t cb) {
    return add_handle(exec, MOCK_ROS_SRV, srv, req, res, (void *)cb, ALWAYS);
}

rcl_ret_t rclc_executor_spin_some(rclc_executor_t *exec, int64_t timeout_ns) {
    (void)exec; (void)timeout_ns;
    mock_spin_calls++;
    if (mock_spin_calls >= (mock_spin_max > 0 ? mock_spin_max : 1)) mock_task_exit();
    return mock_spin_ret;
}

/* ---- rosidl helpers ---- */
bool rosidl_runtime_c__String__assign(rosidl_runtime_c__String *str, const char *value) {
    if (!str || !value) return false;
    size_t n = strlen(value);
    char *p = realloc(str->data, n + 1);
    if (!p) return false;
    memcpy(p, value, n + 1);
    str->data = p;
    str->size = n;
    str->capacity = n + 1;
    return true;
}

void rosidl_runtime_c__String__fini(rosidl_runtime_c__String *str) {
    if (!str) return;
    free(str->data);
    str->data = NULL;
    str->size = str->capacity = 0;
}

bool sensor_msgs__msg__Imu__init(sensor_msgs__msg__Imu *msg) {
    if (!msg) return false;
    memset(msg, 0, sizeof *msg);
    return true;
}

bool actuator_msgs__msg__Actuators__init(actuator_msgs__msg__Actuators *msg) {
    if (!msg) return false;
    memset(msg, 0, sizeof *msg);
    return true;
}

void mock_reset_ros(void) {
    memset(mock_ros_entities, 0, sizeof mock_ros_entities);
    mock_ros_entity_count = 0;
    mock_rcl_calls = 0;
    mock_rcl_fail_at = 0;
    mock_rcl_publish_calls = 0;
    mock_rcl_last_pub = NULL;
    mock_rcl_last_msg = NULL;
    mock_uros_agent_ip[0] = '\0';
    mock_uros_agent_port[0] = '\0';
    memset(mock_exec_handles, 0, sizeof mock_exec_handles);
    mock_exec_handle_count = 0;
    mock_exec_max_handles = 0;
    mock_spin_calls = 0;
    mock_spin_max = 1;
    mock_spin_ret = RCL_RET_OK;
}
