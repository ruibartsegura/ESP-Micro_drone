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
 *   - test_params_*, test_cmdvel_*, test_takeoff_*, test_land_*,
 *     test_state_msg_*, test_task_*, test_init_*, test_sim_*.
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

#define GRAVITY 9.81

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

static std_srvs__srv__Trigger_Response call_land(void) {
    std_srvs__srv__Trigger_Request req = { 0 };
    std_srvs__srv__Trigger_Response res;
    memset(&res, 0, sizeof res);
    res.success = !false;   /* garbage: the callback must write it */
    land_callback(&req, &res);
    return res;
}

static void reset_module(void) {
    is_init = false;
    params_2_update = false;
    take_off_ready = false;
    altitude = MIN_ALT;
    land_ready = false;
    memset(&param_msg, 0, sizeof param_msg);
    memset(&cmd_vel_msg, 0, sizeof cmd_vel_msg);
    memset(&cmd_vel, 0, sizeof cmd_vel);
    t_last_vel = -1;
    memset(&state_msg, 0, sizeof state_msg);
#ifdef CONFIG_SIMULATION_ON
    memset(&imu_sub_msg, 0, sizeof imu_sub_msg);
    memset(&h_sub_msg, 0, sizeof h_sub_msg);
    memset(&motors_msg, 0, sizeof motors_msg);
    memset(motors_buf, 0, sizeof motors_buf);
    if (motors_q) vQueueDelete(motors_q);
    motors_q = NULL;
    motors_pub_ready = false;
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
#define MAX_AGE_MS 500

static void send_cmd_vel(double vx, double vy, double vz, double wz) {
    geometry_msgs__msg__TwistStamped msg = { 0 };
    msg.twist.linear.x = vx;
    msg.twist.linear.y = vy;
    msg.twist.linear.z = vz;
    msg.twist.angular.z = wz;
    cmd_vel_callback(&msg);
}

void test_cmdvel_nothing_received_is_not_fresh(void) {
    mock_time_us = 10000000;   /* 10 s */
    geometry_msgs__msg__Twist t = { 0 };
    TEST_ASSERT_FALSE(get_cmd_vel(&t, MAX_AGE_MS));
}

/* [BUG] t_last_vel starts at -1, but get_cmd_vel() only checks
 * t_last_vel != 0 (the value of clear_cmd_vel()): during the first
 * MAX_AGE_MS after boot it says that a (zero) cmd_vel has arrived. */
void test_cmdvel_nothing_received_just_after_boot_is_not_fresh(void) {
    mock_time_us = 100000;     /* 100 ms after boot */
    geometry_msgs__msg__Twist t = { 0 };
    TEST_ASSERT_FALSE_MESSAGE(get_cmd_vel(&t, MAX_AGE_MS), "no cmd_vel has been received");
}

void test_cmdvel_callback_saves_the_twist(void) {
    mock_time_us = 1000000;
    send_cmd_vel(0.3, -0.2, 0.1, 0.5);
    geometry_msgs__msg__Twist t = { 0 };
    TEST_ASSERT_TRUE(get_cmd_vel(&t, MAX_AGE_MS));
    TEST_ASSERT_EQUAL_DOUBLE(0.3, t.linear.x);
    TEST_ASSERT_EQUAL_DOUBLE(-0.2, t.linear.y);
    TEST_ASSERT_EQUAL_DOUBLE(0.1, t.linear.z);
    TEST_ASSERT_EQUAL_DOUBLE(0.5, t.angular.z);
}

void test_cmdvel_callback_saves_the_time(void) {
    mock_time_us = 1234567;
    send_cmd_vel(0, 0, 0, 0);
    TEST_ASSERT_EQUAL_INT64(1234567, t_last_vel);
}

void test_cmdvel_is_fresh_until_max_age(void) {
    mock_time_us = 1000000;
    send_cmd_vel(0.3, 0, 0, 0);
    geometry_msgs__msg__Twist t = { 0 };
    mock_time_us = 1000000 + MAX_AGE_MS * 1000 - 1;
    TEST_ASSERT_TRUE(get_cmd_vel(&t, MAX_AGE_MS));
    mock_time_us = 1000000 + MAX_AGE_MS * 1000;
    TEST_ASSERT_FALSE(get_cmd_vel(&t, MAX_AGE_MS));
}

void test_cmdvel_max_age_is_the_one_asked(void) {
    mock_time_us = 1000000;
    send_cmd_vel(0.3, 0, 0, 0);
    geometry_msgs__msg__Twist t = { 0 };
    mock_time_us = 1200000;   /* 200 ms later */
    TEST_ASSERT_FALSE(get_cmd_vel(&t, 100));
    TEST_ASSERT_TRUE(get_cmd_vel(&t, 300));
}

void test_cmdvel_old_command_does_not_change_the_output(void) {
    mock_time_us = 1000000;
    send_cmd_vel(0.3, 0, 0, 0);
    mock_time_us = 5000000;
    geometry_msgs__msg__Twist t = { 0 };
    t.linear.x = 9.0;
    TEST_ASSERT_FALSE(get_cmd_vel(&t, MAX_AGE_MS));
    TEST_ASSERT_EQUAL_DOUBLE(9.0, t.linear.x);
}

void test_cmdvel_can_be_read_many_times(void) {
    mock_time_us = 1000000;
    send_cmd_vel(0.3, 0, 0, 0);
    geometry_msgs__msg__Twist t = { 0 };
    TEST_ASSERT_TRUE(get_cmd_vel(&t, MAX_AGE_MS));
    TEST_ASSERT_TRUE(get_cmd_vel(&t, MAX_AGE_MS));
}

void test_cmdvel_new_message_replaces_the_old_one(void) {
    mock_time_us = 1000000;
    send_cmd_vel(0.3, 0, 0, 0);
    mock_time_us = 1400000;
    send_cmd_vel(-0.1, 0, 0, 0);
    mock_time_us = 1800000;   /* 800 ms after the first, 400 ms after the second */
    geometry_msgs__msg__Twist t = { 0 };
    TEST_ASSERT_TRUE(get_cmd_vel(&t, MAX_AGE_MS));
    TEST_ASSERT_EQUAL_DOUBLE(-0.1, t.linear.x);
}

void test_cmdvel_clear_forgets_the_command(void) {
    mock_time_us = 1000000;
    send_cmd_vel(0.3, 0, 0, 0);
    clear_cmd_vel();
    geometry_msgs__msg__Twist t = { 0 };
    TEST_ASSERT_FALSE(get_cmd_vel(&t, MAX_AGE_MS));
    send_cmd_vel(0.2, 0, 0, 0);   /* a new one is fresh again */
    TEST_ASSERT_TRUE(get_cmd_vel(&t, MAX_AGE_MS));
    TEST_ASSERT_EQUAL_DOUBLE(0.2, t.linear.x);
}

/* Call a getter in another thread while this thread holds the mutex:
 * if the getter uses the mutex it must wait. */
static volatile int getter_done;
static void *thread_cmd_vel(void *a) {
    (void)a;
    geometry_msgs__msg__Twist t;
    (void)get_cmd_vel(&t, MAX_AGE_MS);
    getter_done = 1;
    return NULL;
}
static void *thread_clear_cmd_vel(void *a) { (void)a; clear_cmd_vel(); getter_done = 1; return NULL; }
static void *thread_take_off_alt(void *a) { (void)a; (void)get_take_off_alt(); getter_done = 1; return NULL; }
static void *thread_landing_ready(void *a) { (void)a; (void)get_landing_ready(); getter_done = 1; return NULL; }

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

void test_cmdvel_getter_uses_the_mutex(void) {
    TEST_ASSERT_TRUE_MESSAGE(getter_waits_for_the_mutex(thread_cmd_vel),
                             "get_cmd_vel() must lock the mutex");
}

void test_cmdvel_clear_uses_the_mutex(void) {
    TEST_ASSERT_TRUE(getter_waits_for_the_mutex(thread_clear_cmd_vel));
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
/*                      LAND SERVICE (drone/land_srv)                 */
/* ------------------------------------------------------------------ */
void test_land_default_not_ready(void) {
    TEST_ASSERT_FALSE(get_landing_ready());
}

void test_land_accepted_while_flying(void) {
    const sm_states_t flying[] = { HOVERING, EXTERNAL_CONTROL };
    for (size_t i = 0; i < sizeof flying / sizeof flying[0]; i++) {
        set_state(flying[i]);
        std_srvs__srv__Trigger_Response r = call_land();
        TEST_ASSERT_TRUE(r.success);
        TEST_ASSERT_EQUAL_STRING("OK", r.message.data);
        TEST_ASSERT_TRUE(get_landing_ready());
        rosidl_runtime_c__String__fini(&r.message);
    }
}

void test_land_rejected_in_ERROR(void) {
    set_state(ERROR);
    std_srvs__srv__Trigger_Response r = call_land();
    TEST_ASSERT_FALSE(r.success);
    TEST_ASSERT_EQUAL_STRING("Status error", r.message.data);
    TEST_ASSERT_FALSE(get_landing_ready());
    rosidl_runtime_c__String__fini(&r.message);
}

void test_land_rejected_when_not_flying(void) {
    const sm_states_t states[] = { INIT, CHECKING, ARMING, TAKING_OFF, LANDING, DISARMING };
    for (size_t i = 0; i < sizeof states / sizeof states[0]; i++) {
        set_state(states[i]);
        std_srvs__srv__Trigger_Response r = call_land();
        TEST_ASSERT_FALSE(r.success);
        TEST_ASSERT_EQUAL_STRING("Not flying", r.message.data);
        TEST_ASSERT_FALSE(get_landing_ready());
        rosidl_runtime_c__String__fini(&r.message);
    }
}

void test_land_ready_is_read_only_once(void) {
    set_state(HOVERING);
    std_srvs__srv__Trigger_Response r = call_land();
    TEST_ASSERT_TRUE(get_landing_ready());
    TEST_ASSERT_FALSE(get_landing_ready());
    rosidl_runtime_c__String__fini(&r.message);
}

void test_land_getter_uses_the_mutex(void) {
    TEST_ASSERT_TRUE(getter_waits_for_the_mutex(thread_landing_ready));
}

/* ------------------------------------------------------------------ */
/*                      STATE MESSAGE (drone/state)                   */
/* ------------------------------------------------------------------ */
void test_state_msg_angular_velocity_in_rad_per_s(void) {
    my_msgs__msg__State m;
    memset(&m, 0, sizeof m);
    set_vel_ang(90.0f, -45.0f, 180.0f);   /* deg/s */
    fill_state_msg(&m);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, M_PI / 2, m.vel_ang.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, -M_PI / 4, m.vel_ang.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, M_PI, m.vel_ang.z);
}

void test_state_msg_linear_acceleration_in_m_per_s2(void) {
    my_msgs__msg__State m;
    memset(&m, 0, sizeof m);
    set_acc_lin(0.0f, 0.5f, 1.0f);   /* g */
    fill_state_msg(&m);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, 0.0, m.acc_lin.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, 0.5 * GRAVITY, m.acc_lin.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, GRAVITY, m.acc_lin.z);
}

#ifndef CONFIG_SIMULATION_ON
/* Before the time sync the stamp is the time since boot
 * (esp_timer_get_time() is in microseconds). */
void test_state_msg_stamp_from_microseconds(void) {
    my_msgs__msg__State m;
    memset(&m, 0, sizeof m);
    mock_time_us = 2500000;   /* 2.5 s */
    fill_state_msg(&m);
    TEST_ASSERT_EQUAL_INT32(2, m.header.stamp.sec);
    TEST_ASSERT_EQUAL_UINT32(500000000u, m.header.stamp.nanosec);
}

/* After the time sync the stamp is the ROS time of the agent (epoch). */
void test_state_msg_stamp_is_the_agent_time_when_synced(void) {
    my_msgs__msg__State m;
    memset(&m, 0, sizeof m);
    mock_time_us = 2500000;
    mock_uros_synced = true;
    mock_uros_epoch_ns = 1760000000LL * 1000000000LL + 123456789LL;
    fill_state_msg(&m);
    TEST_ASSERT_EQUAL_INT32(1760000000, m.header.stamp.sec);
    TEST_ASSERT_EQUAL_UINT32(123456789u, m.header.stamp.nanosec);
}
#else
/* In simulation the ROS time is the time of Gazebo (last IMU sample). */
void test_state_msg_stamp_is_the_gazebo_time(void) {
    my_msgs__msg__State m;
    memset(&m, 0, sizeof m);
    mock_time_us = 9000000;            /* not used */
    mock_uros_synced = true;           /* not used */
    mock_uros_epoch_ns = 1;
    set_time_imu(2500000000LL);        /* 2.5 s of sim time */
    fill_state_msg(&m);
    TEST_ASSERT_EQUAL_INT32(2, m.header.stamp.sec);
    TEST_ASSERT_EQUAL_UINT32(500000000u, m.header.stamp.nanosec);
}
#endif

void test_state_msg_nanosec_is_always_less_than_1_s(void) {
    my_msgs__msg__State m;
    for (int64_t t = 0; t < 5000000; t += 123457) {
        mock_time_us = t;
        set_time_imu(t * 1000);
        fill_state_msg(&m);
        TEST_ASSERT_TRUE(m.header.stamp.nanosec < 1000000000u);
    }
}

void test_state_msg_position(void) {
    my_msgs__msg__State m;
    memset(&m, 0, sizeof m);
    set_position(1.5f, -2.0f);
    set_h(0.75f);
    fill_state_msg(&m);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 1.5, m.pos.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -2.0, m.pos.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.75, m.pos.z);
}

void test_state_msg_orientation(void) {
    my_msgs__msg__State m;
    memset(&m, 0, sizeof m);
    set_orientation(0.1f, 0.2f, 0.3f, 0.9f);
    fill_state_msg(&m);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.1, m.q.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.2, m.q.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.3, m.q.z);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.9, m.q.w);
}

void test_state_msg_initial_orientation_is_valid(void) {
    my_msgs__msg__State m;
    memset(&m, 0, sizeof m);
    fill_state_msg(&m);   /* state_init(): q = (0, 0, 0, 1) */
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 1.0, m.q.w);
}

void test_state_msg_linear_velocity_in_m_per_s(void) {
    my_msgs__msg__State m;
    memset(&m, 0, sizeof m);
    set_vel_lin(0.5f, -0.25f, 1.0f);
    fill_state_msg(&m);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.5, m.vel_lin.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -0.25, m.vel_lin.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 1.0, m.vel_lin.z);
}

void test_state_msg_sm_state(void) {
    my_msgs__msg__State m;
    memset(&m, 0, sizeof m);
    set_state(HOVERING);
    fill_state_msg(&m);
    TEST_ASSERT_EQUAL_UINT8(my_msgs__msg__State__HOVERING, m.sm_state);
    set_state(ERROR);
    fill_state_msg(&m);
    TEST_ASSERT_EQUAL_UINT8(my_msgs__msg__State__ERROR, m.sm_state);
}

void test_state_msg_sm_state_values_match_the_enum(void) {
    TEST_ASSERT_EQUAL_INT(INIT, my_msgs__msg__State__INIT);
    TEST_ASSERT_EQUAL_INT(CHECKING, my_msgs__msg__State__CHECKING);
    TEST_ASSERT_EQUAL_INT(ARMING, my_msgs__msg__State__ARMING);
    TEST_ASSERT_EQUAL_INT(TAKING_OFF, my_msgs__msg__State__TAKING_OFF);
    TEST_ASSERT_EQUAL_INT(HOVERING, my_msgs__msg__State__HOVERING);
    TEST_ASSERT_EQUAL_INT(EXTERNAL_CONTROL, my_msgs__msg__State__EXTERNAL_CONTROL);
    TEST_ASSERT_EQUAL_INT(LANDING, my_msgs__msg__State__LANDING);
    TEST_ASSERT_EQUAL_INT(DISARMING, my_msgs__msg__State__DISARMING);
    TEST_ASSERT_EQUAL_INT(ERROR, my_msgs__msg__State__ERROR);
}

/* The timer fills the message with the current state before publishing it. */
void test_state_msg_timer_publishes_the_current_state(void) {
    rcl_timer_t t;
    set_state(LANDING);
    set_h(0.4f);
    set_vel_ang(90.0f, 0.0f, 0.0f);
    timer_callback(&t, 0);
    TEST_ASSERT_EQUAL_UINT8(my_msgs__msg__State__LANDING, state_msg.sm_state);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.4, state_msg.pos.z);
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, M_PI / 2, state_msg.vel_ang.x);
}

void test_state_msg_timer_publishes_on_drone_state(void) {
    rcl_timer_t t;
    set_state(HOVERING);
    timer_callback(&t, 0);
    TEST_ASSERT_EQUAL_INT(1, mock_rcl_publish_calls);
    TEST_ASSERT_EQUAL_PTR(&state_pub, mock_rcl_last_pub);
    TEST_ASSERT_EQUAL_PTR(&state_msg, mock_rcl_last_msg);
}

void test_state_msg_timer_null_does_nothing(void) {
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

void test_task_state_publisher(void) {
    run_ros_task();
    const mock_ros_entity_t *e = mock_ros_find(MOCK_ROS_PUB, "drone/state");
    TEST_ASSERT_NOT_NULL(e);
    TEST_ASSERT_EQUAL_STRING("my_msgs/msg/State", e->type);
    TEST_ASSERT_TRUE(e->best_effort);
    TEST_ASSERT_EQUAL_PTR(&state_pub, e->handle);
}

void test_task_no_imu_publisher(void) {
    run_ros_task();
    TEST_ASSERT_NULL(mock_ros_find(MOCK_ROS_PUB, "drone/imu_data"));
}

void test_task_state_message_is_prepared(void) {
    run_ros_task();
    TEST_ASSERT_EQUAL_STRING("odom", state_msg.header.frame_id.data);
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

void test_task_land_service(void) {
    run_ros_task();
    const mock_ros_entity_t *e = mock_ros_find(MOCK_ROS_SRV, "drone/land_srv");
    TEST_ASSERT_NOT_NULL(e);
    TEST_ASSERT_EQUAL_STRING("std_srvs/srv/Trigger", e->type);
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
    h = find_handle(&land_srv);
    TEST_ASSERT_NOT_NULL(h);
    TEST_ASSERT_EQUAL_PTR(&land_req, h->msg);
    TEST_ASSERT_EQUAL_PTR(&land_res, h->res);
    TEST_ASSERT_EQUAL_PTR((void *)land_callback, h->callback);
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
void test_task_syncs_the_time_with_the_agent(void) {
    run_ros_task();
    TEST_ASSERT_EQUAL_INT(1, mock_uros_sync_calls);
    TEST_ASSERT_EQUAL_INT(TIME_SYNC_TIMEOUT_MS, mock_uros_sync_timeout_ms);
    TEST_ASSERT_TRUE(rmw_uros_epoch_synchronized());
}

void test_task_without_agent_does_not_sync_the_time(void) {
    mock_rcl_fail_at = 3;   /* support init fails */
    run_ros_task();
    TEST_ASSERT_EQUAL_INT(0, mock_uros_sync_calls);
}

/* If the sync fails the task goes on (stamps with the time since boot). */
void test_task_time_sync_error_does_not_stop_the_task(void) {
    mock_uros_sync_ret = 1;   /* RMW_RET_ERROR */
    run_ros_task();
    TEST_ASSERT_EQUAL_INT(1, mock_uros_sync_calls);
    TEST_ASSERT_EQUAL_INT(0, mock_task_delete_calls);
    TEST_ASSERT_EQUAL_INT(1, mock_spin_calls);
    TEST_ASSERT_FALSE(rmw_uros_epoch_synchronized());
}

void test_task_resyncs_the_time_every_60_s(void) {
    TEST_ASSERT_EQUAL_INT64(60000000LL, TIME_SYNC_PERIOD_US);
    mock_time_step_us = 30000000;   /* every esp_timer_get_time() is 30 s later */
    mock_spin_max = 5;              /* 4 loops: t = 60, 90, 120, 150 s */
    mock_run_task(micro_ros_task, NULL, 1000);
    TEST_ASSERT_EQUAL_INT(3, mock_uros_sync_calls);   /* start, 60 s, 120 s */
}

void test_task_does_not_resync_before_60_s(void) {
    mock_time_step_us = 1000000;    /* 1 s */
    mock_spin_max = 30;
    mock_run_task(micro_ros_task, NULL, 1000);
    TEST_ASSERT_EQUAL_INT(1, mock_uros_sync_calls);
}

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

/* In simulation the time comes from Gazebo: no sync with the agent. */
void test_sim_task_does_not_sync_the_time(void) {
    mock_time_step_us = 30000000;
    mock_spin_max = 5;
    mock_run_task(micro_ros_task, NULL, 1000);
    TEST_ASSERT_EQUAL_INT(0, mock_uros_sync_calls);
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

/* BEST_EFFORT: the Gazebo bridge subscribes with SENSOR_DATA (best effort,
 * see ros_ws/src/sim_drone/config/sim_bridge.yaml). If the bridge goes back
 * to RELIABLE, a BEST_EFFORT publisher is not received. */
void test_sim_motors_publisher_is_best_effort(void) {
    run_ros_task();
    const mock_ros_entity_t *e = mock_ros_find(MOCK_ROS_PUB, "/drone/command/motor_speed");
    TEST_ASSERT_NOT_NULL(e);
    TEST_ASSERT_EQUAL_STRING("actuator_msgs/msg/Actuators", e->type);
    TEST_ASSERT_TRUE(e->best_effort);
    TEST_ASSERT_TRUE(motors_pub_ready);
}

void test_sim_motors_publisher_session_timeout_is_50_ms(void) {
    run_ros_task();
    TEST_ASSERT_EQUAL_PTR(&motors_pub, mock_uros_pub_timeout_pub);
    TEST_ASSERT_EQUAL_INT(50, mock_uros_pub_timeout_ms);
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
    set_time_imu(7250000000LL);   /* Gazebo time: 7.25 s */
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

    RUN_TEST(test_cmdvel_nothing_received_is_not_fresh);
    RUN_TEST(test_cmdvel_nothing_received_just_after_boot_is_not_fresh);
    RUN_TEST(test_cmdvel_callback_saves_the_twist);
    RUN_TEST(test_cmdvel_callback_saves_the_time);
    RUN_TEST(test_cmdvel_is_fresh_until_max_age);
    RUN_TEST(test_cmdvel_max_age_is_the_one_asked);
    RUN_TEST(test_cmdvel_old_command_does_not_change_the_output);
    RUN_TEST(test_cmdvel_can_be_read_many_times);
    RUN_TEST(test_cmdvel_new_message_replaces_the_old_one);
    RUN_TEST(test_cmdvel_clear_forgets_the_command);
    RUN_TEST(test_cmdvel_take_off_alt_getter_uses_the_mutex);
    RUN_TEST(test_cmdvel_getter_uses_the_mutex);
    RUN_TEST(test_cmdvel_clear_uses_the_mutex);

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

    RUN_TEST(test_land_default_not_ready);
    RUN_TEST(test_land_accepted_while_flying);
    RUN_TEST(test_land_rejected_in_ERROR);
    RUN_TEST(test_land_rejected_when_not_flying);
    RUN_TEST(test_land_ready_is_read_only_once);
    RUN_TEST(test_land_getter_uses_the_mutex);

    RUN_TEST(test_state_msg_angular_velocity_in_rad_per_s);
    RUN_TEST(test_state_msg_linear_acceleration_in_m_per_s2);
#ifndef CONFIG_SIMULATION_ON
    RUN_TEST(test_state_msg_stamp_from_microseconds);
    RUN_TEST(test_state_msg_stamp_is_the_agent_time_when_synced);
#else
    RUN_TEST(test_state_msg_stamp_is_the_gazebo_time);
#endif
    RUN_TEST(test_state_msg_nanosec_is_always_less_than_1_s);
    RUN_TEST(test_state_msg_position);
    RUN_TEST(test_state_msg_orientation);
    RUN_TEST(test_state_msg_initial_orientation_is_valid);
    RUN_TEST(test_state_msg_linear_velocity_in_m_per_s);
    RUN_TEST(test_state_msg_sm_state);
    RUN_TEST(test_state_msg_sm_state_values_match_the_enum);
    RUN_TEST(test_state_msg_timer_publishes_the_current_state);
    RUN_TEST(test_state_msg_timer_publishes_on_drone_state);
    RUN_TEST(test_state_msg_timer_null_does_nothing);

    RUN_TEST(test_task_uses_the_agent_from_menuconfig);
    RUN_TEST(test_task_creates_the_node_rui_drone);
    RUN_TEST(test_task_turns_on_the_esp_led_when_connected);
    RUN_TEST(test_task_state_publisher);
    RUN_TEST(test_task_no_imu_publisher);
    RUN_TEST(test_task_state_message_is_prepared);
    RUN_TEST(test_task_params_subscriber);
    RUN_TEST(test_task_cmd_vel_subscriber);
    RUN_TEST(test_task_takeoff_service);
    RUN_TEST(test_task_land_service);
    RUN_TEST(test_task_main_timer_is_1_s);
    RUN_TEST(test_task_handles_go_to_the_right_callbacks);
    RUN_TEST(test_task_every_entity_is_in_the_executor);
    RUN_TEST(test_task_executor_has_enough_handles);
    RUN_TEST(test_task_spins_the_executor);
    RUN_TEST(test_task_any_init_error_stops_the_task);
    RUN_TEST(test_task_no_agent_no_led);
#ifndef CONFIG_SIMULATION_ON
    RUN_TEST(test_task_syncs_the_time_with_the_agent);
    RUN_TEST(test_task_without_agent_does_not_sync_the_time);
    RUN_TEST(test_task_time_sync_error_does_not_stop_the_task);
    RUN_TEST(test_task_resyncs_the_time_every_60_s);
    RUN_TEST(test_task_does_not_resync_before_60_s);
    RUN_TEST(test_task_hardware_has_no_simulation_topics);
#endif

    RUN_TEST(test_init_test_is_false_before_init);
    RUN_TEST(test_init_creates_the_task_with_the_menuconfig_values);
    RUN_TEST(test_init_starts_the_network_and_disables_wifi_power_save);
    RUN_TEST(test_init_twice_is_done_once);
    RUN_TEST(test_init_network_error_is_checked);

#ifdef CONFIG_SIMULATION_ON
    RUN_TEST(test_sim_has_8_handles);
    RUN_TEST(test_sim_task_does_not_sync_the_time);
    RUN_TEST(test_sim_imu_subscriber);
    RUN_TEST(test_sim_barometer_subscriber);
    RUN_TEST(test_sim_motors_publisher_is_best_effort);
    RUN_TEST(test_sim_motors_publisher_session_timeout_is_50_ms);
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
