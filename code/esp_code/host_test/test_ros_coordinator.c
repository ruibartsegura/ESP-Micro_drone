/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for ros_coordinator.c (host, no hardware, no ROS). The file
 *   is built twice: ros_coordinator_hw and ros_coordinator_sim
 *   (CONFIG_SIMULATION_ON, with the extra subscribers of the IMU and the
 *   barometer and the publisher of the motors).
 *
 *   micro-ROS (rcl, rclc, executor) is mocked: every publisher,
 *   subscriber, service, timer and executor handle is saved, so the tests
 *   check the names, types and QoS of the ROS interface, the number of
 *   executor handles (N_HANDLERS) and what happens when an init call fails.
 *   The callbacks are called directly, like the executor does.
 *   The real state.c, parameters.c and bmp180.c are linked.
 *
 *   Units: the global state uses deg/s and g (like the IMU driver);
 *   sensor_msgs/Imu uses rad/s and m/s^2; ROS times are sec + nanosec.
 *
 *   The .c file is included directly. Another version can be tested with:
 *       make run T=ros_coordinator_hw MODULE_SRC=path/to/ros_coordinator.c
 *
 *   Tests that fail with the current code have [BUG] in their comment.
 *
 * Functions:
 *   - led_*(): mocks of led.c.
 *   - run_ros_task(): runs micro_ros_task() until the first spin.
 *   - find_handle(): looks for an executor handle.
 *   - setUp() / tearDown(): reset the mocks, the module and the state.
 *   - test_params_*, test_cmdvel_*, test_takeoff_*, test_imu_msg_*,
 *     test_task_*, test_init_*, test_sim_*.
 *   - main(): runs all the tests.
 */
#include <pthread.h>
#include <unistd.h>
#include "unity.h"
#include "test_helpers.h"
#include "mocks/mock_reset.h"

#ifndef MODULE_SRC
#define MODULE_SRC "../components/ros_coordinator/ros_coordinator.c"
#endif
#include MODULE_SRC

#define GRAVITY 9.80665

/* ------------------------------------------------------------------ */
/*                           MOCKS of led.c                           */
/* ------------------------------------------------------------------ */
static int led_on_calls[N_LEDS];
void led_init(void) {}
bool led_test(void) { return true; }
void led_on(led_t led)  { led_on_calls[led]++; }
void led_off(led_t led) { (void)led; }
void all_on(void) {}
void all_off(void) {}

/* ------------------------------------------------------------------ */
/*                              HELPERS                               */
/* ------------------------------------------------------------------ */
static void run_ros_task(void) {
    mock_spin_max = 1;
    mock_run_task(micro_ros_task, NULL, 1000);
}

static const mock_exec_handle_t *find_handle(const void *entity) {
    for (int i = 0; i < mock_exec_handle_count; i++)
        if (mock_exec_handles[i].entity == entity) return &mock_exec_handles[i];
    return NULL;
}

static void set_state(sm_states_t s) { set_sm_state(s); }

static my_msgs__srv__Takeoff_Response call_takeoff(float alt) {
    my_msgs__srv__Takeoff_Request req = { .altitude = alt };
    my_msgs__srv__Takeoff_Response res;
    memset(&res, 0, sizeof res);
    res.accepted = !false;   /* garbage: the callback must write it */
    takeoff_callback(&req, &res);
    return res;
}

static void reset_module(void) {
    is_init = false;
    params_2_update = false;
    take_off_ready = false;
    altitude = MIN_ALT;
    memset(&param_msg, 0, sizeof param_msg);
    memset(&cmd_vel_msg, 0, sizeof cmd_vel_msg);
    memset(&imu_msg, 0, sizeof imu_msg);
#ifdef CONFIG_SIMULATION_ON
    memset(&imu_sub_msg, 0, sizeof imu_sub_msg);
    memset(&h_sub_msg, 0, sizeof h_sub_msg);
    memset(&motors_msg, 0, sizeof motors_msg);
    memset(motors_buf, 0, sizeof motors_buf);
    if (motors_q) vQueueDelete(motors_q);
    motors_q = NULL;
    motors_pub_ready = false;
    imu_recv_cnt = 0;
#endif
}

void setUp(void) {
    mock_reset_all();
    reset_module();
    state_init();
    memset(led_on_calls, 0, sizeof led_on_calls);
    set_hovering_h(0.5f);
    set_max_velocity(1.0f);
    set_land_on_site(true);
}

void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                        PARAMETERS (drone/params)                   */
/* ------------------------------------------------------------------ */
static void send_params(float h, float v, bool land) {
    my_msgs__msg__Params p = { .h_max = h, .v_max = v, .land_on_site = land };
    param_callback(&p);
}

void test_params_callback_does_not_apply_them_yet(void) {
    send_params(1.2f, 2.5f, false);
    TEST_ASSERT_TRUE(params_2_update);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, get_hovering_h());
}

void test_params_apply_sets_every_parameter(void) {
    send_params(1.2f, 2.5f, false);
    apply_pending_params();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.2f, get_hovering_h());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.5f, get_max_velocity());
    TEST_ASSERT_FALSE(get_land_on_site());
    TEST_ASSERT_FALSE(params_2_update);
}

void test_params_apply_without_new_message_does_nothing(void) {
    apply_pending_params();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, get_hovering_h());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, get_max_velocity());
}

void test_params_last_message_wins(void) {
    send_params(1.0f, 1.0f, true);
    send_params(2.0f, 3.0f, false);
    apply_pending_params();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, get_hovering_h());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.0f, get_max_velocity());
}

void test_params_timer_applies_them_in_CHECKING(void) {
    rcl_timer_t t;
    set_state(CHECKING);
    send_params(1.5f, 2.0f, true);
    timer_callback(&t, 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.5f, get_hovering_h());
}

void test_params_timer_does_not_apply_them_while_flying(void) {
    const sm_states_t flying[] = { ARMING, TAKING_OFF, HOVERING, EXTERNAL_CONTROL, LANDING };
    rcl_timer_t t;
    for (size_t i = 0; i < sizeof flying / sizeof flying[0]; i++) {
        set_hovering_h(0.5f);
        set_state(flying[i]);
        send_params(2.5f, 2.0f, true);
        timer_callback(&t, 0);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, get_hovering_h());
        TEST_ASSERT_TRUE_MESSAGE(params_2_update, "the parameters must stay pending");
    }
}

/* ------------------------------------------------------------------ */
/*                         CMD_VEL (drone/cmd_vel)                    */
/* ------------------------------------------------------------------ */
void test_cmdvel_default_is_zero(void) {
    geometry_msgs__msg__Twist t = get_cmd_vel();
    TEST_ASSERT_EQUAL_DOUBLE(0.0, t.linear.x);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, t.angular.z);
}

void test_cmdvel_callback_saves_the_twist(void) {
    geometry_msgs__msg__TwistStamped msg = { 0 };
    msg.twist.linear.x = 0.3;
    msg.twist.linear.y = -0.2;
    msg.twist.linear.z = 0.1;
    msg.twist.angular.z = 0.5;
    cmd_vel_callback(&msg);
    geometry_msgs__msg__Twist t = get_cmd_vel();
    TEST_ASSERT_EQUAL_DOUBLE(0.3, t.linear.x);
    TEST_ASSERT_EQUAL_DOUBLE(-0.2, t.linear.y);
    TEST_ASSERT_EQUAL_DOUBLE(0.1, t.linear.z);
    TEST_ASSERT_EQUAL_DOUBLE(0.5, t.angular.z);
}

/* Call a getter in another thread while this thread holds the mutex:
 * if the getter uses the mutex it must wait. */
static volatile int getter_done;
static void *thread_cmd_vel(void *a) { (void)a; (void)get_cmd_vel(); getter_done = 1; return NULL; }
static void *thread_take_off_alt(void *a) { (void)a; (void)get_take_off_alt(); getter_done = 1; return NULL; }

static bool getter_waits_for_the_mutex(void *(*fn)(void *)) {
    pthread_t th;
    getter_done = 0;
    pthread_mutex_lock(&lock);
    pthread_create(&th, NULL, fn, NULL);
    usleep(100000);   /* 100 ms */
    bool waited = !getter_done;
    pthread_mutex_unlock(&lock);
    pthread_join(th, NULL);
    return waited;
}

void test_cmdvel_take_off_alt_getter_uses_the_mutex(void) {
    TEST_ASSERT_TRUE(getter_waits_for_the_mutex(thread_take_off_alt));
}

/* [BUG] get_cmd_vel() reads cmd_vel_msg without the mutex, while the
 * executor task can be writing it in cmd_vel_callback(): the controller can
 * read half of an old command and half of a new one. */
void test_cmdvel_getter_uses_the_mutex(void) {
    TEST_ASSERT_TRUE_MESSAGE(getter_waits_for_the_mutex(thread_cmd_vel),
                             "get_cmd_vel() must lock the mutex");
}

/* ------------------------------------------------------------------ */
/*                    TAKE-OFF SERVICE (drone/takeoff_srv)            */
/* ------------------------------------------------------------------ */
void test_takeoff_default_not_ready_and_min_altitude(void) {
    TEST_ASSERT_FALSE(get_take_off_ready());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, MIN_ALT, get_take_off_alt());
}

void test_takeoff_limits_are_0_5_and_3_m(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, MIN_ALT);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.0f, MAX_ALT);
}

void test_takeoff_accepted_in_ARMING(void) {
    set_state(ARMING);
    my_msgs__srv__Takeoff_Response r = call_takeoff(1.5f);
    TEST_ASSERT_TRUE(r.accepted);
    TEST_ASSERT_EQUAL_STRING("OK", r.reason.data);
    TEST_ASSERT_TRUE(get_take_off_ready());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.5f, get_take_off_alt());
    rosidl_runtime_c__String__fini(&r.reason);
}

void test_takeoff_rejected_in_ERROR(void) {
    set_state(ERROR);
    my_msgs__srv__Takeoff_Response r = call_takeoff(1.0f);
    TEST_ASSERT_FALSE(r.accepted);
    TEST_ASSERT_EQUAL_STRING("Status error", r.reason.data);
    TEST_ASSERT_FALSE(get_take_off_ready());
    rosidl_runtime_c__String__fini(&r.reason);
}

void test_takeoff_rejected_in_every_state_except_ARMING(void) {
    const sm_states_t states[] = { INIT, CHECKING, TAKING_OFF, HOVERING, EXTERNAL_CONTROL, LANDING, DISARMING };
    for (size_t i = 0; i < sizeof states / sizeof states[0]; i++) {
        set_state(states[i]);
        my_msgs__srv__Takeoff_Response r = call_takeoff(1.0f);
        TEST_ASSERT_FALSE(r.accepted);
        TEST_ASSERT_EQUAL_STRING("Not armed", r.reason.data);
        TEST_ASSERT_FALSE(get_take_off_ready());
        rosidl_runtime_c__String__fini(&r.reason);
    }
}

void test_takeoff_rejected_out_of_range(void) {
    const float bad[] = { 0.49f, 3.01f, 0.0f, -1.0f, 100.0f };
    set_state(ARMING);
    for (size_t i = 0; i < sizeof bad / sizeof bad[0]; i++) {
        my_msgs__srv__Takeoff_Response r = call_takeoff(bad[i]);
        TEST_ASSERT_FALSE(r.accepted);
        TEST_ASSERT_EQUAL_STRING("Invalid altitude", r.reason.data);
        TEST_ASSERT_FALSE(get_take_off_ready());
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, MIN_ALT, get_take_off_alt());
        rosidl_runtime_c__String__fini(&r.reason);
    }
}

void test_takeoff_limits_are_accepted(void) {
    set_state(ARMING);
    my_msgs__srv__Takeoff_Response r1 = call_takeoff(0.5f);
    TEST_ASSERT_TRUE(r1.accepted);
    my_msgs__srv__Takeoff_Response r2 = call_takeoff(3.0f);
    TEST_ASSERT_TRUE(r2.accepted);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.0f, get_take_off_alt());
    rosidl_runtime_c__String__fini(&r1.reason);
    rosidl_runtime_c__String__fini(&r2.reason);
}

/* [BUG] The range check uses "alt < MIN || alt > MAX", which is false for
 * NaN: a NaN altitude is accepted and the controller gets a NaN target. */
void test_takeoff_nan_altitude_is_rejected(void) {
    set_state(ARMING);
    my_msgs__srv__Takeoff_Response r = call_takeoff(NAN);
    TEST_ASSERT_FALSE_MESSAGE(r.accepted, "NaN must not be a valid altitude");
    TEST_ASSERT_FALSE(get_take_off_ready());
    rosidl_runtime_c__String__fini(&r.reason);
}

void test_takeoff_ready_is_read_only_once(void) {
    set_state(ARMING);
    my_msgs__srv__Takeoff_Response r = call_takeoff(1.0f);
    TEST_ASSERT_TRUE(get_take_off_ready());
    TEST_ASSERT_FALSE(get_take_off_ready());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, get_take_off_alt());   /* the altitude stays */
    rosidl_runtime_c__String__fini(&r.reason);
}

void test_takeoff_new_request_changes_the_altitude(void) {
    set_state(ARMING);
    my_msgs__srv__Takeoff_Response r1 = call_takeoff(1.0f);
    my_msgs__srv__Takeoff_Response r2 = call_takeoff(2.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, get_take_off_alt());
    rosidl_runtime_c__String__fini(&r1.reason);
    rosidl_runtime_c__String__fini(&r2.reason);
}

void test_takeoff_rejected_request_keeps_the_previous_one(void) {
    set_state(ARMING);
    my_msgs__srv__Takeoff_Response r1 = call_takeoff(1.0f);
    my_msgs__srv__Takeoff_Response r2 = call_takeoff(9.0f);
    TEST_ASSERT_TRUE(get_take_off_ready());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, get_take_off_alt());
    rosidl_runtime_c__String__fini(&r1.reason);
    rosidl_runtime_c__String__fini(&r2.reason);
}

/* ------------------------------------------------------------------ */
/*                       IMU MESSAGE (drone/imu_data)                 */
/* ------------------------------------------------------------------ */
/* [BUG] fill_imu_msg() first writes angular_velocity in rad/s and then
 * overwrites it with vel_ang * GRAVITY_MS2 (it should be
 * linear_acceleration = acc_lin * GRAVITY_MS2). */
void test_imu_msg_angular_velocity_in_rad_per_s(void) {
    sensor_msgs__msg__Imu m;
    memset(&m, 0, sizeof m);
    set_vel_ang(90.0f, -45.0f, 180.0f);   /* deg/s */
    fill_imu_msg(&m);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, M_PI / 2, m.angular_velocity.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, -M_PI / 4, m.angular_velocity.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, M_PI, m.angular_velocity.z);
}

/* [BUG] The linear acceleration is never written (see the test above). */
void test_imu_msg_linear_acceleration_in_m_per_s2(void) {
    sensor_msgs__msg__Imu m;
    memset(&m, 0, sizeof m);
    set_acc_lin(0.0f, 0.5f, 1.0f);   /* g */
    fill_imu_msg(&m);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, 0.0, m.linear_acceleration.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, 0.5 * GRAVITY, m.linear_acceleration.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, GRAVITY, m.linear_acceleration.z);
}

/* [BUG] esp_timer_get_time() returns microseconds, but fill_imu_msg()
 * treats it as milliseconds: after 2.5 s the stamp says 2500 s. */
void test_imu_msg_stamp_from_microseconds(void) {
    sensor_msgs__msg__Imu m;
    memset(&m, 0, sizeof m);
    mock_time_us = 2500000;   /* 2.5 s */
    fill_imu_msg(&m);
    TEST_ASSERT_EQUAL_INT32(2, m.header.stamp.sec);
    TEST_ASSERT_EQUAL_UINT32(500000000u, m.header.stamp.nanosec);
}

void test_imu_msg_nanosec_is_always_less_than_1_s(void) {
    sensor_msgs__msg__Imu m;
    for (int64_t t = 0; t < 5000000; t += 123457) {
        mock_time_us = t;
        fill_imu_msg(&m);
        TEST_ASSERT_TRUE(m.header.stamp.nanosec < 1000000000u);
    }
}

void test_imu_msg_timer_publishes_on_drone_imu_data(void) {
    rcl_timer_t t;
    set_state(HOVERING);
    timer_callback(&t, 0);
    TEST_ASSERT_EQUAL_INT(1, mock_rcl_publish_calls);
    TEST_ASSERT_EQUAL_PTR(&imu_pub, mock_rcl_last_pub);
    TEST_ASSERT_EQUAL_PTR(&imu_msg, mock_rcl_last_msg);
}

void test_imu_msg_timer_null_does_nothing(void) {
    timer_callback(NULL, 0);
    TEST_ASSERT_EQUAL_INT(0, mock_rcl_publish_calls);
}

/* ------------------------------------------------------------------ */
/*                    micro_ros_task (node, topics, executor)         */
/* ------------------------------------------------------------------ */
void test_task_uses_the_agent_from_menuconfig(void) {
    run_ros_task();
    TEST_ASSERT_EQUAL_STRING(CONFIG_MICRO_ROS_AGENT_IP, mock_uros_agent_ip);
    TEST_ASSERT_EQUAL_STRING(CONFIG_MICRO_ROS_AGENT_PORT, mock_uros_agent_port);
}

void test_task_creates_the_node_rui_drone(void) {
    run_ros_task();
    TEST_ASSERT_EQUAL_INT(1, mock_ros_count(MOCK_ROS_NODE));
    TEST_ASSERT_NOT_NULL(mock_ros_find(MOCK_ROS_NODE, "rui_drone"));
}

void test_task_turns_on_the_esp_led_when_connected(void) {
    run_ros_task();
    TEST_ASSERT_EQUAL_INT(1, led_on_calls[LED_ESP]);
}

void test_task_imu_publisher(void) {
    run_ros_task();
    const mock_ros_entity_t *e = mock_ros_find(MOCK_ROS_PUB, "drone/imu_data");
    TEST_ASSERT_NOT_NULL(e);
    TEST_ASSERT_EQUAL_STRING("sensor_msgs/msg/Imu", e->type);
    TEST_ASSERT_TRUE(e->best_effort);
    TEST_ASSERT_EQUAL_PTR(&imu_pub, e->handle);
}

void test_task_imu_message_is_prepared(void) {
    run_ros_task();
    TEST_ASSERT_EQUAL_STRING("imu_link", imu_msg.header.frame_id.data);
    TEST_ASSERT_EQUAL_DOUBLE_MESSAGE(-1.0, imu_msg.orientation_covariance[0],
                                     "-1 = no orientation (REP 145)");
}

void test_task_params_subscriber(void) {
    run_ros_task();
    const mock_ros_entity_t *e = mock_ros_find(MOCK_ROS_SUB, "drone/params");
    TEST_ASSERT_NOT_NULL(e);
    TEST_ASSERT_EQUAL_STRING("my_msgs/msg/Params", e->type);
    TEST_ASSERT_TRUE(e->has_qos);
    TEST_ASSERT_EQUAL_INT(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, e->qos.reliability);
    TEST_ASSERT_EQUAL_INT(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, e->qos.durability);
    TEST_ASSERT_EQUAL_INT(RMW_QOS_POLICY_HISTORY_KEEP_LAST, e->qos.history);
    TEST_ASSERT_EQUAL_UINT(2, e->qos.depth);
}

void test_task_cmd_vel_subscriber(void) {
    run_ros_task();
    const mock_ros_entity_t *e = mock_ros_find(MOCK_ROS_SUB, "drone/cmd_vel");
    TEST_ASSERT_NOT_NULL(e);
    TEST_ASSERT_EQUAL_STRING("geometry_msgs/msg/TwistStamped", e->type);
    TEST_ASSERT_TRUE(e->best_effort);
}

void test_task_takeoff_service(void) {
    run_ros_task();
    const mock_ros_entity_t *e = mock_ros_find(MOCK_ROS_SRV, "drone/takeoff_srv");
    TEST_ASSERT_NOT_NULL(e);
    TEST_ASSERT_EQUAL_STRING("my_msgs/srv/Takeoff", e->type);
}

void test_task_main_timer_is_1_s(void) {
    run_ros_task();
    bool found = false;
    for (int i = 0; i < mock_ros_entity_count; i++)
        if (mock_ros_entities[i].kind == MOCK_ROS_TIMER && mock_ros_entities[i].timer_cb == timer_callback) {
            found = true;
            TEST_ASSERT_EQUAL_INT64(RCL_MS_TO_NS(1000), mock_ros_entities[i].period_ns);
        }
    TEST_ASSERT_TRUE(found);
}

void test_task_handles_go_to_the_right_callbacks(void) {
    run_ros_task();
    const mock_exec_handle_t *h;
    h = find_handle(&params_sub);
    TEST_ASSERT_NOT_NULL(h);
    TEST_ASSERT_EQUAL_PTR(&recv_msg, h->msg);
    TEST_ASSERT_EQUAL_PTR((void *)param_callback, h->callback);
    h = find_handle(&cmd_vel_sub);
    TEST_ASSERT_NOT_NULL(h);
    TEST_ASSERT_EQUAL_PTR(&cmd_vel_msg, h->msg);
    TEST_ASSERT_EQUAL_PTR((void *)cmd_vel_callback, h->callback);
    h = find_handle(&takeoff_srv);
    TEST_ASSERT_NOT_NULL(h);
    TEST_ASSERT_EQUAL_PTR(&takeoff_req, h->msg);
    TEST_ASSERT_EQUAL_PTR(&takeoff_res, h->res);
    TEST_ASSERT_EQUAL_PTR((void *)takeoff_callback, h->callback);
}

void test_task_every_entity_is_in_the_executor(void) {
    run_ros_task();
    int subs = mock_ros_count(MOCK_ROS_SUB), srvs = mock_ros_count(MOCK_ROS_SRV), timers = mock_ros_count(MOCK_ROS_TIMER);
    TEST_ASSERT_EQUAL_INT(subs + srvs + timers, mock_exec_handle_count);
}

void test_task_executor_has_enough_handles(void) {
    run_ros_task();
    TEST_ASSERT_EQUAL_UINT(N_HANDLERS, mock_exec_max_handles);
    TEST_ASSERT_TRUE((size_t)mock_exec_handle_count <= mock_exec_max_handles);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, mock_task_delete_calls, "the init failed");
}

void test_task_spins_the_executor(void) {
    run_ros_task();
    TEST_ASSERT_EQUAL_INT(1, mock_spin_calls);
}

void test_task_any_init_error_stops_the_task(void) {
    run_ros_task();
    int n = mock_rcl_calls;   /* calls of a good init */
    TEST_ASSERT_TRUE(n > 5);
    for (int k = 1; k <= n; k++) {
        setUp();
        mock_rcl_fail_at = k;
        run_ros_task();
        char msg[64];
        snprintf(msg, sizeof msg, "failure in call %d of the init", k);
        TEST_ASSERT_EQUAL_INT_MESSAGE(1, mock_task_delete_calls, msg);
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, mock_spin_calls, msg);
    }
}

void test_task_no_agent_no_led(void) {
    mock_rcl_fail_at = 3;   /* init options, udp address, support init */
    run_ros_task();
    TEST_ASSERT_EQUAL_INT(0, led_on_calls[LED_ESP]);
    TEST_ASSERT_EQUAL_INT(0, mock_ros_count(MOCK_ROS_NODE));
}

#ifndef CONFIG_SIMULATION_ON
void test_task_hardware_has_no_simulation_topics(void) {
    run_ros_task();
    TEST_ASSERT_NULL(mock_ros_find(MOCK_ROS_SUB, "imu/data"));
    TEST_ASSERT_NULL(mock_ros_find(MOCK_ROS_SUB, "barometer/data"));
    TEST_ASSERT_NULL(mock_ros_find(MOCK_ROS_PUB, "/drone/command/motor_speed"));
    TEST_ASSERT_EQUAL_INT(5, N_HANDLERS);
}
#endif

/* ------------------------------------------------------------------ */
/*                              ros_init                              */
/* ------------------------------------------------------------------ */
void test_init_test_is_false_before_init(void) {
    TEST_ASSERT_FALSE(ros_test());
}

void test_init_creates_the_task_with_the_menuconfig_values(void) {
    ros_init();
    const mock_task_t *t = mock_find_task("uros_task");
    TEST_ASSERT_NOT_NULL(t);
    TEST_ASSERT_EQUAL_PTR((void *)micro_ros_task, (void *)t->fn);
    TEST_ASSERT_EQUAL_UINT32(CONFIG_MICRO_ROS_STACK, t->stack);
    TEST_ASSERT_EQUAL_UINT(CONFIG_MICRO_ROS_TASK_PRIO, t->prio);
    TEST_ASSERT_TRUE(ros_test());
}

void test_init_starts_the_network_and_disables_wifi_power_save(void) {
    ros_init();
    TEST_ASSERT_EQUAL_INT(1, mock_netif_init_calls);
    TEST_ASSERT_EQUAL_INT(1, mock_wifi_set_ps_calls);
    TEST_ASSERT_EQUAL_INT(WIFI_PS_NONE, mock_wifi_ps_mode);
    TEST_ASSERT_EQUAL_INT(0, mock_esp_error_check_fails);
}

void test_init_twice_is_done_once(void) {
    ros_init();
    ros_init();
    TEST_ASSERT_EQUAL_INT(1, mock_task_count);
    TEST_ASSERT_EQUAL_INT(1, mock_netif_init_calls);
}

void test_init_network_error_is_checked(void) {
    mock_netif_init_ret = ESP_FAIL;
    ros_init();
    TEST_ASSERT_EQUAL_INT(1, mock_esp_error_check_fails);
}

/* ------------------------------------------------------------------ */
/*                           SIMULATION ONLY                          */
/* ------------------------------------------------------------------ */
#ifdef CONFIG_SIMULATION_ON
void test_sim_has_8_handles(void) {
    TEST_ASSERT_EQUAL_INT(8, N_HANDLERS);
}

void test_sim_imu_subscriber(void) {
    run_ros_task();
    const mock_ros_entity_t *e = mock_ros_find(MOCK_ROS_SUB, "imu/data");
    TEST_ASSERT_NOT_NULL(e);
    TEST_ASSERT_EQUAL_STRING("sensor_msgs/msg/Imu", e->type);
    TEST_ASSERT_EQUAL_INT(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, e->qos.reliability);
    TEST_ASSERT_EQUAL_UINT(1, e->qos.depth);
    const mock_exec_handle_t *h = find_handle(&imu_sub);
    TEST_ASSERT_NOT_NULL(h);
    TEST_ASSERT_EQUAL_PTR((void *)imu_callback, h->callback);
}

void test_sim_barometer_subscriber(void) {
    run_ros_task();
    const mock_ros_entity_t *e = mock_ros_find(MOCK_ROS_SUB, "barometer/data");
    TEST_ASSERT_NOT_NULL(e);
    TEST_ASSERT_EQUAL_STRING("sensor_msgs/msg/FluidPressure", e->type);
    TEST_ASSERT_EQUAL_INT(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, e->qos.reliability);
    TEST_ASSERT_EQUAL_UINT(1, e->qos.depth);
    const mock_exec_handle_t *h = find_handle(&height_sub);
    TEST_ASSERT_NOT_NULL(h);
    TEST_ASSERT_EQUAL_PTR((void *)height_callback, h->callback);
}

void test_sim_motors_publisher_is_reliable(void) {
    /* the Gazebo bridge subscribes with RELIABLE: a BEST_EFFORT publisher
     * would not be received (see CLAUDE.md, QoS of the motors) */
    run_ros_task();
    const mock_ros_entity_t *e = mock_ros_find(MOCK_ROS_PUB, "/drone/command/motor_speed");
    TEST_ASSERT_NOT_NULL(e);
    TEST_ASSERT_EQUAL_STRING("actuator_msgs/msg/Actuators", e->type);
    TEST_ASSERT_FALSE(e->best_effort);
    TEST_ASSERT_EQUAL_INT(RMW_QOS_POLICY_RELIABILITY_RELIABLE, e->qos.reliability);
    TEST_ASSERT_EQUAL_INT(RMW_QOS_POLICY_HISTORY_KEEP_LAST, e->qos.history);
    TEST_ASSERT_EQUAL_UINT(2, e->qos.depth);
    TEST_ASSERT_TRUE(motors_pub_ready);
}

void test_sim_motors_timer_is_30_ms(void) {
    run_ros_task();
    bool found = false;
    for (int i = 0; i < mock_ros_entity_count; i++)
        if (mock_ros_entities[i].kind == MOCK_ROS_TIMER && mock_ros_entities[i].timer_cb == motors_timer_callback) {
            found = true;
            TEST_ASSERT_EQUAL_INT64(RCL_MS_TO_NS(30), mock_ros_entities[i].period_ns);
        }
    TEST_ASSERT_TRUE(found);
}

static sensor_msgs__msg__Imu gz_imu(double wx, double wy, double wz, double ax, double ay, double az,
                                    int32_t sec, uint32_t nsec) {
    sensor_msgs__msg__Imu m;
    memset(&m, 0, sizeof m);
    m.angular_velocity.x = wx; m.angular_velocity.y = wy; m.angular_velocity.z = wz;
    m.linear_acceleration.x = ax; m.linear_acceleration.y = ay; m.linear_acceleration.z = az;
    m.header.stamp.sec = sec;
    m.header.stamp.nanosec = nsec;
    return m;
}

void test_sim_imu_callback_saves_the_message(void) {
    sensor_msgs__msg__Imu m = gz_imu(0.1, 0.2, 0.3, 0, 0, GRAVITY, 1, 0);
    imu_callback(&m);
    TEST_ASSERT_EQUAL_DOUBLE(0.3, imu_sub_msg.angular_velocity.z);
    TEST_ASSERT_EQUAL_UINT32(1, imu_recv_cnt);
}

/* [BUG] The real IMU driver saves deg/s in the global state and the
 * attitude controller integrates deg/s. Gazebo sends rad/s and the
 * callback saves it without conversion: in simulation the gyro looks
 * 57 times slower. */
void test_sim_imu_callback_angular_velocity_in_deg_per_s(void) {
    sensor_msgs__msg__Imu m = gz_imu(M_PI / 2, 0, -M_PI, 0, 0, GRAVITY, 0, 0);
    imu_callback(&m);
    vec3_t w;
    get_vel_ang(&w);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 90.0f, w.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -180.0f, w.z);
}

/* [BUG] Same for the acceleration: the driver saves g, Gazebo sends m/s^2. */
void test_sim_imu_callback_acceleration_in_g(void) {
    sensor_msgs__msg__Imu m = gz_imu(0, 0, 0, 0, GRAVITY / 2, GRAVITY, 0, 0);
    imu_callback(&m);
    vec3_t a;
    get_acc_lin(&a);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5f, a.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, a.z);
}

/* [BUG] Only header.stamp.nanosec is saved, the seconds are lost: the time
 * goes back to 0 every second and the controller sees dt < 0 (it ignores
 * the gyro in that cycle). The time must be sec * 1e9 + nanosec. */
void test_sim_imu_callback_time_in_nanoseconds(void) {
    sensor_msgs__msg__Imu m = gz_imu(0, 0, 0, 0, 0, GRAVITY, 12, 345);
    imu_callback(&m);
    int64_t t;
    get_time_imu(&t);
    TEST_ASSERT_EQUAL_INT64(12000000345LL, t);
}

/* [BUG] Consequence of the bug above: the time must always grow. */
void test_sim_imu_callback_time_always_grows(void) {
    int64_t prev = -1;
    for (int i = 0; i < 300; i++) {   /* 3 s at 100 Hz */
        int64_t ns = (int64_t)i * 10000000LL;
        sensor_msgs__msg__Imu m = gz_imu(0, 0, 0, 0, 0, GRAVITY, (int32_t)(ns / 1000000000LL), (uint32_t)(ns % 1000000000LL));
        imu_callback(&m);
        int64_t t;
        get_time_imu(&t);
        TEST_ASSERT_TRUE_MESSAGE(t > prev, "the IMU time went back");
        prev = t;
    }
}

void test_sim_height_callback_saves_the_altitude(void) {
    sensor_msgs__msg__FluidPressure m;
    memset(&m, 0, sizeof m);
    m.fluid_pressure = 100000.0;   /* Pa -> 110.9 m */
    height_callback(&m);
    vec3_t p;
    get_position(&p);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 110.9f, p.z);
}

void test_sim_height_callback_sea_level_is_zero(void) {
    sensor_msgs__msg__FluidPressure m;
    memset(&m, 0, sizeof m);
    m.fluid_pressure = 101325.0;
    height_callback(&m);
    vec3_t p;
    get_position(&p);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, p.z);
}

/* [BUG] The pressure is converted to an integer (int32_t) before computing
 * the altitude: 1 Pa is ~8 cm, so the height jumps in steps of 8 cm. */
void test_sim_height_callback_keeps_the_decimals_of_the_pressure(void) {
    sensor_msgs__msg__FluidPressure m;
    memset(&m, 0, sizeof m);
    m.fluid_pressure = 101324.5;   /* half a pascal: ~4 cm */
    height_callback(&m);
    vec3_t p;
    get_position(&p);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.042f, p.z);
}

/* [BUG] Same time problem as the IMU: the seconds are lost. */
void test_sim_height_callback_time_in_nanoseconds(void) {
    sensor_msgs__msg__FluidPressure m;
    memset(&m, 0, sizeof m);
    m.fluid_pressure = 101325.0;
    m.header.stamp.sec = 3;
    m.header.stamp.nanosec = 500;
    height_callback(&m);
    int64_t t;
    get_time_height(&t);
    TEST_ASSERT_EQUAL_INT64(3000000500LL, t);
}

void test_sim_motors_message_is_prepared(void) {
    motors_msg_init();
    TEST_ASSERT_EQUAL_PTR(motors_buf, motors_msg.velocity.data);
    TEST_ASSERT_EQUAL_UINT(NUM_MOTORS, motors_msg.velocity.size);
    TEST_ASSERT_EQUAL_UINT(NUM_MOTORS, motors_msg.velocity.capacity);
    TEST_ASSERT_EQUAL_STRING("base_link", motors_msg.header.frame_id.data);
    TEST_ASSERT_NOT_NULL(motors_q);
}

void test_sim_ros_init_prepares_the_motors(void) {
    ros_init();
    TEST_ASSERT_NOT_NULL(motors_q);
}

void test_sim_pub_motor_speed_before_init_is_ignored(void) {
    double p[NUM_MOTORS] = { 1, 2, 3, 4 };
    TEST_ASSERT_NO_CRASH(pub_motor_speed(p), "pub_motor_speed() before ros_init()");
}

void test_sim_motors_timer_publishes_the_last_command(void) {
    rcl_timer_t t;
    motors_msg_init();
    motors_pub_ready = true;
    double p1[NUM_MOTORS] = { 1, 2, 3, 4 };
    double p2[NUM_MOTORS] = { 10, 20, 30, 40 };
    pub_motor_speed(p1);
    pub_motor_speed(p2);   /* overwrites the first one */
    mock_time_us = 7250000;
    motors_timer_callback(&t, 0);
    TEST_ASSERT_EQUAL_INT(1, mock_rcl_publish_calls);
    TEST_ASSERT_EQUAL_PTR(&motors_pub, mock_rcl_last_pub);
    for (int i = 0; i < NUM_MOTORS; i++) TEST_ASSERT_EQUAL_DOUBLE(p2[i], motors_msg.velocity.data[i]);
    TEST_ASSERT_EQUAL_INT32(7, motors_msg.header.stamp.sec);
    TEST_ASSERT_EQUAL_UINT32(250000000u, motors_msg.header.stamp.nanosec);
}

void test_sim_motors_timer_publishes_each_command_once(void) {
    rcl_timer_t t;
    motors_msg_init();
    motors_pub_ready = true;
    double p[NUM_MOTORS] = { 1, 1, 1, 1 };
    pub_motor_speed(p);
    motors_timer_callback(&t, 0);
    motors_timer_callback(&t, 0);
    TEST_ASSERT_EQUAL_INT(1, mock_rcl_publish_calls);
}

void test_sim_motors_timer_nothing_new_nothing_published(void) {
    rcl_timer_t t;
    motors_msg_init();
    motors_pub_ready = true;
    motors_timer_callback(&t, 0);
    TEST_ASSERT_EQUAL_INT(0, mock_rcl_publish_calls);
}

void test_sim_motors_timer_waits_for_the_publisher(void) {
    rcl_timer_t t;
    motors_msg_init();
    double p[NUM_MOTORS] = { 1, 1, 1, 1 };
    pub_motor_speed(p);
    motors_timer_callback(&t, 0);   /* publisher not ready */
    TEST_ASSERT_EQUAL_INT(0, mock_rcl_publish_calls);
    motors_pub_ready = true;
    motors_timer_callback(&t, 0);   /* the command is still there */
    TEST_ASSERT_EQUAL_INT(1, mock_rcl_publish_calls);
}
#endif

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_params_callback_does_not_apply_them_yet);
    RUN_TEST(test_params_apply_sets_every_parameter);
    RUN_TEST(test_params_apply_without_new_message_does_nothing);
    RUN_TEST(test_params_last_message_wins);
    RUN_TEST(test_params_timer_applies_them_in_CHECKING);
    RUN_TEST(test_params_timer_does_not_apply_them_while_flying);

    RUN_TEST(test_cmdvel_default_is_zero);
    RUN_TEST(test_cmdvel_callback_saves_the_twist);
    RUN_TEST(test_cmdvel_take_off_alt_getter_uses_the_mutex);
    RUN_TEST(test_cmdvel_getter_uses_the_mutex);

    RUN_TEST(test_takeoff_default_not_ready_and_min_altitude);
    RUN_TEST(test_takeoff_limits_are_0_5_and_3_m);
    RUN_TEST(test_takeoff_accepted_in_ARMING);
    RUN_TEST(test_takeoff_rejected_in_ERROR);
    RUN_TEST(test_takeoff_rejected_in_every_state_except_ARMING);
    RUN_TEST(test_takeoff_rejected_out_of_range);
    RUN_TEST(test_takeoff_limits_are_accepted);
    RUN_TEST(test_takeoff_nan_altitude_is_rejected);
    RUN_TEST(test_takeoff_ready_is_read_only_once);
    RUN_TEST(test_takeoff_new_request_changes_the_altitude);
    RUN_TEST(test_takeoff_rejected_request_keeps_the_previous_one);

    RUN_TEST(test_imu_msg_angular_velocity_in_rad_per_s);
    RUN_TEST(test_imu_msg_linear_acceleration_in_m_per_s2);
    RUN_TEST(test_imu_msg_stamp_from_microseconds);
    RUN_TEST(test_imu_msg_nanosec_is_always_less_than_1_s);
    RUN_TEST(test_imu_msg_timer_publishes_on_drone_imu_data);
    RUN_TEST(test_imu_msg_timer_null_does_nothing);

    RUN_TEST(test_task_uses_the_agent_from_menuconfig);
    RUN_TEST(test_task_creates_the_node_rui_drone);
    RUN_TEST(test_task_turns_on_the_esp_led_when_connected);
    RUN_TEST(test_task_imu_publisher);
    RUN_TEST(test_task_imu_message_is_prepared);
    RUN_TEST(test_task_params_subscriber);
    RUN_TEST(test_task_cmd_vel_subscriber);
    RUN_TEST(test_task_takeoff_service);
    RUN_TEST(test_task_main_timer_is_1_s);
    RUN_TEST(test_task_handles_go_to_the_right_callbacks);
    RUN_TEST(test_task_every_entity_is_in_the_executor);
    RUN_TEST(test_task_executor_has_enough_handles);
    RUN_TEST(test_task_spins_the_executor);
    RUN_TEST(test_task_any_init_error_stops_the_task);
    RUN_TEST(test_task_no_agent_no_led);
#ifndef CONFIG_SIMULATION_ON
    RUN_TEST(test_task_hardware_has_no_simulation_topics);
#endif

    RUN_TEST(test_init_test_is_false_before_init);
    RUN_TEST(test_init_creates_the_task_with_the_menuconfig_values);
    RUN_TEST(test_init_starts_the_network_and_disables_wifi_power_save);
    RUN_TEST(test_init_twice_is_done_once);
    RUN_TEST(test_init_network_error_is_checked);

#ifdef CONFIG_SIMULATION_ON
    RUN_TEST(test_sim_has_8_handles);
    RUN_TEST(test_sim_imu_subscriber);
    RUN_TEST(test_sim_barometer_subscriber);
    RUN_TEST(test_sim_motors_publisher_is_reliable);
    RUN_TEST(test_sim_motors_timer_is_30_ms);
    RUN_TEST(test_sim_imu_callback_saves_the_message);
    RUN_TEST(test_sim_imu_callback_angular_velocity_in_deg_per_s);
    RUN_TEST(test_sim_imu_callback_acceleration_in_g);
    RUN_TEST(test_sim_imu_callback_time_in_nanoseconds);
    RUN_TEST(test_sim_imu_callback_time_always_grows);
    RUN_TEST(test_sim_height_callback_saves_the_altitude);
    RUN_TEST(test_sim_height_callback_sea_level_is_zero);
    RUN_TEST(test_sim_height_callback_keeps_the_decimals_of_the_pressure);
    RUN_TEST(test_sim_height_callback_time_in_nanoseconds);
    RUN_TEST(test_sim_motors_message_is_prepared);
    RUN_TEST(test_sim_ros_init_prepares_the_motors);
    RUN_TEST(test_sim_pub_motor_speed_before_init_is_ignored);
    RUN_TEST(test_sim_motors_timer_publishes_the_last_command);
    RUN_TEST(test_sim_motors_timer_publishes_each_command_once);
    RUN_TEST(test_sim_motors_timer_nothing_new_nothing_published);
    RUN_TEST(test_sim_motors_timer_waits_for_the_publisher);
#endif

    return UNITY_END();
}
