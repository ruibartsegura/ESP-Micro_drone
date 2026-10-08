/* Mock of rclc/executor.h (host tests). See rcl/rcl.h. */
#pragma once
#include "rclc/rclc.h"

typedef enum { ON_NEW_DATA = 0, ALWAYS } rclc_executor_handle_invocation_t;
typedef void (*rclc_subscription_callback_t)(const void *);
typedef void (*rclc_service_callback_t)(const void *, void *);

typedef struct { size_t max_handles; size_t used; int initialised; } rclc_executor_t;

typedef struct {
    mock_ros_kind_t kind;          /* MOCK_ROS_SUB, MOCK_ROS_SRV or MOCK_ROS_TIMER */
    const void     *entity;
    void           *msg;           /* subscription msg / service request          */
    void           *res;           /* service response                            */
    void           *callback;
    rclc_executor_handle_invocation_t invocation;
} mock_exec_handle_t;

extern mock_exec_handle_t mock_exec_handles[MOCK_ROS_MAX];
extern int                mock_exec_handle_count;
extern size_t             mock_exec_max_handles;   /* value given to rclc_executor_init */
extern int                mock_spin_calls;
extern int                mock_spin_max;           /* stop the task after N spins (0 = 1) */
extern rcl_ret_t          mock_spin_ret;

rcl_ret_t rclc_executor_init(rclc_executor_t *exec, rcl_context_t *ctx, size_t handles, const rcl_allocator_t *alloc);
rcl_ret_t rclc_executor_add_timer(rclc_executor_t *exec, rcl_timer_t *timer);
rcl_ret_t rclc_executor_add_subscription(rclc_executor_t *exec, rcl_subscription_t *sub, void *msg,
                                         rclc_subscription_callback_t cb,
                                         rclc_executor_handle_invocation_t invocation);
rcl_ret_t rclc_executor_add_service(rclc_executor_t *exec, rcl_service_t *srv, void *req, void *res,
                                    rclc_service_callback_t cb);
rcl_ret_t rclc_executor_spin_some(rclc_executor_t *exec, int64_t timeout_ns);
