/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for system.c, the state machine of the drone (host, no
 *   hardware). The file is built twice: system_hw and system_sim
 *   (CONFIG_SIMULATION_ON, where the IMU and the barometer are not started).
 *   Every module that system.c uses (LEDs, IMU, height, motors, attitude
 *   controller, micro-ROS) is mocked and saves its calls in a log, so the
 *   tests check the init order, the self test and every transition of:
 *     INIT -> CHECKING -> ARMING -> TAKING_OFF -> HOVERING <-> EXTERNAL_CONTROL
 *     -> LANDING -> DISARMING, and ERROR.
 *   The real state.c is linked (the state is saved there).
 *
 *   The .c file is included directly. Another version can be tested with:
 *       make run T=system_hw MODULE_SRC=path/to/system.c
 *
 *   Tests that fail with the current code have [BUG] in their comment.
 *
 * Functions:
 *   - Mocks of led, imu, height, motors, attitude_controller, odometry,
 *     tuning, ros_coordinator.
 *   - called() / call_index(): look in the log of calls.
 *   - go_to(): puts the state machine in a state.
 *   - setUp() / tearDown(): reset the mocks, the module and the state.
 *   - test_init_*, test_selftest_*, test_sm_*, test_task_*.
 *   - main(): runs all the tests.
 */
#include "unity.h"
#include "test_helpers.h"
#include "mocks/mock_reset.h"

#ifndef MODULE_SRC
#define MODULE_SRC "../components/system/system.c"
#endif
#include MODULE_SRC

/* ------------------------------------------------------------------ */
/*                       MOCKS (with a call log)                      */
/* ------------------------------------------------------------------ */
#define LOG_MAX 256
static const char *call_log[LOG_MAX];
static int         n_calls;
static void log_call(const char *name) { if (n_calls < LOG_MAX) call_log[n_calls++] = name; }

static int called(const char *name) {
    int n = 0;
    for (int i = 0; i < n_calls; i++) n += (strcmp(call_log[i], name) == 0);
    return n;
}

static int call_index(const char *name) {
    for (int i = 0; i < n_calls; i++) if (strcmp(call_log[i], name) == 0) return i;
    return -1;
}

static bool led_test_ret, imu_test_ret, height_test_ret, ros_test_ret, motors_test_ret;
static bool take_off_ready_ret, landing_ready_ret, h_reached_ret;
static float target_altitude_ret;
static geometry_msgs__msg__Twist cmd_vel_ret;
static bool cmd_vel_fresh_ret;      /* get_cmd_vel(): a new cmd_vel has arrived */
static int  cmd_vel_max_age_ms;     /* last max_age_ms asked to get_cmd_vel() */
static int led_on_count[N_LEDS], led_off_count[N_LEDS];

void led_init(void)        { log_call("led_init"); }
bool led_test(void)        { log_call("led_test"); return led_test_ret; }
void led_on(led_t led)     { log_call("led_on");  led_on_count[led]++; }
void led_off(led_t led)    { log_call("led_off"); led_off_count[led]++; }
void all_on(void)          { log_call("all_on"); }
void all_off(void)         { log_call("all_off"); }
void imu_init(void)        { log_call("imu_init"); }
bool imu_test(void)        { log_call("imu_test"); return imu_test_ret; }
void height_init(void)     { log_call("height_init"); }
bool height_test(void)     { log_call("height_test"); return height_test_ret; }
void init_motors(void)     { log_call("init_motors"); }
bool motors_test(void)     { log_call("motors_test"); return motors_test_ret; }
void arm_motors(void)      { log_call("arm_motors"); }
void disarm_motors(void)   { log_call("disarm_motors"); }
void motors_stop_all(void) { log_call("motors_stop_all"); }
static double motor_power_ret[N_MOTORS];
void set_motor_speed(double power[N_MOTORS]) {
    log_call("set_motor_speed");
    memcpy(motor_power_ret, power, sizeof motor_power_ret);
}
static float h_est_ret;
static float last_motor_power_ret;
float get_h_est(void) { return h_est_ret; }
float get_last_motor_power(void) { return last_motor_power_ret; }
void init_attitude_controller(void) { log_call("init_attitude_controller"); }
void init_tuning(void)     { log_call("init_tuning"); }
void ros_init(void)        { log_call("ros_init"); }
bool ros_test(void)        { log_call("ros_test"); return ros_test_ret; }
bool get_take_off_ready(void) { log_call("get_take_off_ready"); return take_off_ready_ret; }
float get_target_altitude(void) { log_call("get_target_altitude"); return target_altitude_ret; }
bool get_landing_ready(void)  { log_call("get_landing_ready"); return landing_ready_ret; }
bool check_h_reached(void)    { log_call("check_h_reached"); return h_reached_ret; }
/* Like the real one: the flag is false until the controller sets it again. */
void reset_h_reached(void)    { log_call("reset_h_reached"); h_reached_ret = false; }
void init_odom_estimator(void) { log_call("init_odom_estimator"); }
bool odom_estimator_test(void) { log_call("odom_estimator_test"); return true; }
bool attitude_controller_test(void) { log_call("attitude_controller_test"); return true; }
static int64_t t_last_link_ret;
int64_t get_t_last_link(void) { log_call("get_t_last_link"); return t_last_link_ret; }

bool get_cmd_vel(geometry_msgs__msg__Twist *out, int max_age_ms) {
    log_call("get_cmd_vel");
    cmd_vel_max_age_ms = max_age_ms;
    if (cmd_vel_fresh_ret) {
        *out = cmd_vel_ret;
    }
    return cmd_vel_fresh_ret;
}

/* ------------------------------------------------------------------ */
/*                              HELPERS                               */
/* ------------------------------------------------------------------ */
static sm_states_t global_sm(void) {
    sm_states_t s;
    get_sm_state(&s);
    return s;
}

/* Sensors (and link with the agent) with data right now. A test can stop
 * any of them. */
static bool imu_alive, baro_alive, link_alive;

static void feed_sensors(void) {
    if (imu_alive) {
        set_time_imu(mock_time_us * 1000LL);
    }
    if (baro_alive) {
        set_time_height(mock_time_us * 1000LL);
    }
    if (link_alive) {
        t_last_link_ret = mock_time_us;
    }
}

static void go_to(sm_states_t s) {
    change_state(s);
    n_calls = 0;
}

static void clear_logs(void) {
    n_calls = 0;
    memset(led_on_count, 0, sizeof led_on_count);
    memset(led_off_count, 0, sizeof led_off_count);
}

void setUp(void) {
    mock_reset_all();
    is_init = false;
    state = INIT;
    memset(&att_target, 0, sizeof att_target);
    state_init();
    clear_logs();
    led_test_ret = imu_test_ret = height_test_ret = ros_test_ret = motors_test_ret = true;
    take_off_ready_ret = landing_ready_ret = h_reached_ret = false;
    target_altitude_ret = 1.0f;
    memset(&cmd_vel_ret, 0, sizeof cmd_vel_ret);
    cmd_vel_fresh_ret = false;
    cmd_vel_max_age_ms = -1;
    blink_led = true;
    t_blink_us = -1;
    mock_time_us = 1000000;   /* 1 s */
    f_type = SENSORS_OK;
    f_emergency = SENSORS_OK;
    t_emergency_us = 0;
    emergency_time_us = 0;
    t_imu_cut_us = -1;
    imu_cut_start = 0.0f;
    h_est_ret = 0.0f;
    last_motor_power_ret = 0.0f;
    memset(motor_power_ret, 0, sizeof motor_power_ret);
    imu_alive = baro_alive = link_alive = true;
    feed_sensors();
}

/* One cycle of the state machine, ms later (esp_timer for the LEDs). */
static void cycle_after_ms(int ms) {
    mock_time_us += ms * 1000LL;
    feed_sensors();
    state_machine();
}

void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                              INIT                                  */
/* ------------------------------------------------------------------ */
void test_init_starts_every_module(void) {
    system_init();
    TEST_ASSERT_EQUAL_INT(1, called("led_init"));
    TEST_ASSERT_EQUAL_INT(1, called("init_motors"));
    TEST_ASSERT_EQUAL_INT(1, called("init_attitude_controller"));
    TEST_ASSERT_EQUAL_INT(1, called("init_odom_estimator"));
    TEST_ASSERT_EQUAL_INT(1, called("init_tuning"));   /* CONFIG_GAINS_TUNE_ON */
    TEST_ASSERT_EQUAL_INT(1, called("ros_init"));
#ifdef CONFIG_SIMULATION_ON
    TEST_ASSERT_EQUAL_INT(0, called("imu_init"));
    TEST_ASSERT_EQUAL_INT(0, called("height_init"));
#else
    TEST_ASSERT_EQUAL_INT(1, called("imu_init"));
    TEST_ASSERT_EQUAL_INT(1, called("height_init"));
#endif
}

void test_init_micro_ros_is_the_last_one(void) {
    system_init();
    TEST_ASSERT_EQUAL_INT(n_calls - 1, call_index("ros_init"));
}

void test_init_sensors_before_the_controller(void) {
    system_init();
#ifndef CONFIG_SIMULATION_ON
    TEST_ASSERT_TRUE(call_index("imu_init") < call_index("init_attitude_controller"));
    TEST_ASSERT_TRUE(call_index("height_init") < call_index("init_attitude_controller"));
#endif
    TEST_ASSERT_TRUE(call_index("init_motors") < call_index("init_attitude_controller"));
}

void test_init_resets_the_global_state(void) {
    set_h(5.0f);
    system_init();
    vec3_t p;
    get_position(&p);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, p.z);
}

void test_init_twice_starts_the_modules_once(void) {
    system_init();
    system_init();
    TEST_ASSERT_EQUAL_INT(1, called("led_init"));
    TEST_ASSERT_EQUAL_INT(1, called("ros_init"));
}

/* The odometry uses the vertical velocity of the attitude controller. */
void test_init_odometry_after_the_controller(void) {
    system_init();
    TEST_ASSERT_TRUE(call_index("init_attitude_controller") < call_index("init_odom_estimator"));
}

/* ------------------------------------------------------------------ */
/*                           SELF TEST                                */
/* ------------------------------------------------------------------ */
void test_selftest_false_before_init(void) {
    TEST_ASSERT_FALSE(system_test());
}

void test_selftest_true_when_everything_is_ok(void) {
    system_init();
    TEST_ASSERT_TRUE(system_test());
}

void test_selftest_fails_if_the_leds_fail(void) {
    system_init();
    led_test_ret = false;
    TEST_ASSERT_FALSE(system_test());
}

void test_selftest_fails_if_micro_ros_fails(void) {
    system_init();
    ros_test_ret = false;
    TEST_ASSERT_FALSE(system_test());
}

#ifdef CONFIG_SIMULATION_ON
void test_selftest_simulation_does_not_test_the_sensors(void) {
    system_init();
    imu_test_ret = height_test_ret = false;
    TEST_ASSERT_TRUE(system_test());
    TEST_ASSERT_EQUAL_INT(0, called("imu_test"));
    TEST_ASSERT_EQUAL_INT(0, called("height_test"));
}
#else
void test_selftest_fails_if_the_imu_fails(void) {
    system_init();
    imu_test_ret = false;
    TEST_ASSERT_FALSE(system_test());
}

void test_selftest_fails_if_the_barometer_fails(void) {
    system_init();
    height_test_ret = false;
    TEST_ASSERT_FALSE(system_test());
}
#endif

/* CAUTION: the motor test spins the motors, it runs with every boot. */
void test_selftest_runs_the_motor_test(void) {
    system_init();
    system_test();
    TEST_ASSERT_EQUAL_INT(1, called("motors_test"));
}

void test_selftest_fails_if_the_motors_fail(void) {
    system_init();
    motors_test_ret = false;
    TEST_ASSERT_FALSE(system_test());
}

/* ------------------------------------------------------------------ */
/*                         STATE MACHINE                              */
/* ------------------------------------------------------------------ */
void test_sm_change_state_updates_the_global_state(void) {
    change_state(HOVERING);
    TEST_ASSERT_EQUAL_INT(HOVERING, state);
    TEST_ASSERT_EQUAL_INT(HOVERING, global_sm());
}

void test_sm_starts_in_INIT(void) {
    TEST_ASSERT_EQUAL_INT(INIT, state);
}

void test_sm_INIT_starts_the_system_and_goes_to_CHECKING(void) {
    state_machine();
    TEST_ASSERT_TRUE(is_init);
    TEST_ASSERT_EQUAL_INT(CHECKING, state);
    TEST_ASSERT_EQUAL_INT(CHECKING, global_sm());
}

void test_sm_CHECKING_ok_goes_to_ARMING_with_green_led(void) {
    system_init();
    go_to(CHECKING);
    state_machine();
    TEST_ASSERT_EQUAL_INT(ARMING, state);
    TEST_ASSERT_EQUAL_INT(1, led_on_count[LED_GREEN]);
}

void test_sm_CHECKING_fail_goes_to_ERROR(void) {
    system_init();
    go_to(CHECKING);
    led_test_ret = false;
    state_machine();
    TEST_ASSERT_EQUAL_INT(ERROR, state);
    TEST_ASSERT_EQUAL_INT(0, led_on_count[LED_GREEN]);
}

void test_sm_ARMING_waits_for_the_takeoff(void) {
    go_to(ARMING);
    for (int i = 0; i < 10; i++) state_machine();
    TEST_ASSERT_EQUAL_INT(ARMING, state);
    TEST_ASSERT_EQUAL_INT(0, called("arm_motors"));
    TEST_ASSERT_EQUAL_INT(10, called("get_take_off_ready"));
}

/* The state machine never waits inside a state: the task period (20 ms)
 * is the only wait, so it checks the service every cycle. */
void test_sm_ARMING_does_not_wait_inside_the_state(void) {
    go_to(ARMING);
    state_machine();
    TEST_ASSERT_EQUAL_UINT64(0, mock_delay_total_ms);
}

void test_sm_ARMING_with_takeoff_arms_and_goes_to_TAKING_OFF(void) {
    go_to(ARMING);
    take_off_ready_ret = true;
    state_machine();
    TEST_ASSERT_EQUAL_INT(1, called("arm_motors"));
    TEST_ASSERT_EQUAL_INT(TAKING_OFF, state);
}

void test_sm_TAKING_OFF_targets_the_takeoff_altitude_without_moving(void) {
    go_to(TAKING_OFF);
    target_altitude_ret = 1.5f;
    cmd_vel_ret.linear.x = 3.0;   /* must be ignored */
    cmd_vel_fresh_ret = true;
    state_machine();
    ATTITUDE_TARGET t = get_attitude();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.5f, t.h);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, t.cmd_vel.linear.x);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, t.cmd_vel.angular.z);
}

void test_sm_TAKING_OFF_stays_while_the_height_is_not_reached(void) {
    go_to(TAKING_OFF);
    h_reached_ret = false;
    for (int i = 0; i < 10; i++) state_machine();
    TEST_ASSERT_EQUAL_INT(TAKING_OFF, state);
}

void test_sm_TAKING_OFF_blinks_the_blue_led(void) {
    go_to(TAKING_OFF);
    for (int i = 0; i < 4; i++) cycle_after_ms(BLINK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(2, led_on_count[LED_BLUE]);
    TEST_ASSERT_EQUAL_INT(2, led_off_count[LED_BLUE]);
    TEST_ASSERT_EQUAL_UINT64(0, mock_delay_total_ms);   /* blinks by time, no wait */
}

/* At 20 ms per cycle the LED only changes every BLINK_PERIOD_MS. */
void test_sm_blink_changes_the_led_every_blink_period(void) {
    go_to(TAKING_OFF);
    for (int i = 0; i < 15; i++) cycle_after_ms(SYSTEM_TASK_PERIOD_MS);   /* 300 ms */
    TEST_ASSERT_EQUAL_INT(2, led_on_count[LED_BLUE] + led_off_count[LED_BLUE]);
}

void test_sm_EXTERNAL_CONTROL_blinks_the_green_led_without_waiting(void) {
    go_to(EXTERNAL_CONTROL);
    cmd_vel_fresh_ret = true;
    for (int i = 0; i < 4; i++) cycle_after_ms(BLINK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(EXTERNAL_CONTROL, state);
    TEST_ASSERT_EQUAL_INT(2, led_off_count[LED_GREEN]);
    TEST_ASSERT_EQUAL_UINT64(0, mock_delay_total_ms);
}

void test_sm_TAKING_OFF_height_reached_goes_to_HOVERING_with_blue_led(void) {
    go_to(TAKING_OFF);
    h_reached_ret = true;
    state_machine();
    TEST_ASSERT_EQUAL_INT(HOVERING, state);
    TEST_ASSERT_EQUAL_INT(1, led_on_count[LED_BLUE]);
    TEST_ASSERT_EQUAL_INT(0, led_off_count[LED_BLUE]);
}

/* check_h_reached() is computed by the attitude task with the previous
 * target: in ARMING the target is 0 m and the drone is on the ground, so it
 * is already true. ARMING calls reset_h_reached() before TAKING_OFF, so the
 * first cycle of TAKING_OFF does not go to HOVERING on the ground. */
void test_sm_TAKING_OFF_does_not_trust_the_height_of_the_previous_target(void) {
    go_to(ARMING);
    take_off_ready_ret = true;
    h_reached_ret = true;   /* true from the ground (target 0 m) */
    state_machine();        /* ARMING -> TAKING_OFF */
    state_machine();        /* first cycle of TAKING_OFF */
    TEST_ASSERT_EQUAL_INT_MESSAGE(TAKING_OFF, state, "the first cycle sets the new target, it cannot be reached yet");
}

void test_sm_HOVERING_keeps_the_altitude_and_does_not_move(void) {
    go_to(HOVERING);
    target_altitude_ret = 0.8f;
    cmd_vel_ret.linear.y = 1.0;   /* not fresh: ignored */
    state_machine();
    ATTITUDE_TARGET t = get_attitude();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.8f, t.h);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, t.cmd_vel.linear.y);
    TEST_ASSERT_EQUAL_INT(HOVERING, state);
}

void test_sm_HOVERING_new_cmd_vel_goes_to_EXTERNAL_CONTROL(void) {
    go_to(HOVERING);
    cmd_vel_fresh_ret = true;
    state_machine();
    TEST_ASSERT_EQUAL_INT(EXTERNAL_CONTROL, state);
    TEST_ASSERT_EQUAL_INT(CMD_VEL_EXPIRE_TIME, cmd_vel_max_age_ms);
}

void test_sm_HOVERING_land_goes_to_LANDING(void) {
    go_to(HOVERING);
    landing_ready_ret = true;
    state_machine();
    TEST_ASSERT_EQUAL_INT(LANDING, state);
    TEST_ASSERT_EQUAL_INT(1, led_on_count[LED_GREEN]);
    TEST_ASSERT_EQUAL_INT(1, led_off_count[LED_BLUE]);
}

void test_sm_HOVERING_land_has_priority_over_cmd_vel(void) {
    go_to(HOVERING);
    cmd_vel_fresh_ret = true;
    landing_ready_ret = true;
    state_machine();
    TEST_ASSERT_EQUAL_INT(LANDING, state);
}

void test_sm_EXTERNAL_CONTROL_follows_cmd_vel(void) {
    go_to(EXTERNAL_CONTROL);
    target_altitude_ret = 0.8f;
    cmd_vel_ret.linear.x = 0.5;
    cmd_vel_ret.angular.z = 0.2;
    cmd_vel_fresh_ret = true;
    state_machine();
    ATTITUDE_TARGET t = get_attitude();
    TEST_ASSERT_EQUAL_DOUBLE(0.5, t.cmd_vel.linear.x);
    TEST_ASSERT_EQUAL_DOUBLE(0.2, t.cmd_vel.angular.z);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.8f, t.h);
    TEST_ASSERT_EQUAL_INT(EXTERNAL_CONTROL, state);
}

void test_sm_EXTERNAL_CONTROL_uses_the_cmd_vel_expire_time(void) {
    go_to(EXTERNAL_CONTROL);
    cmd_vel_fresh_ret = true;
    state_machine();
    TEST_ASSERT_EQUAL_INT(CMD_VEL_EXPIRE_TIME, cmd_vel_max_age_ms);
}

void test_sm_EXTERNAL_CONTROL_old_cmd_vel_goes_to_HOVERING(void) {
    go_to(EXTERNAL_CONTROL);
    cmd_vel_ret.linear.x = 0.5;
    cmd_vel_fresh_ret = true;
    state_machine();
    cmd_vel_fresh_ret = false;    /* no cmd_vel for CMD_VEL_EXPIRE_TIME */
    state_machine();
    ATTITUDE_TARGET t = get_attitude();
    TEST_ASSERT_EQUAL_INT(HOVERING, state);
    TEST_ASSERT_EQUAL_DOUBLE_MESSAGE(0.0, t.cmd_vel.linear.x, "without cmd_vel the drone must stop");
}

void test_sm_EXTERNAL_CONTROL_land_goes_to_LANDING(void) {
    go_to(EXTERNAL_CONTROL);
    cmd_vel_fresh_ret = true;
    landing_ready_ret = true;
    state_machine();
    TEST_ASSERT_EQUAL_INT(LANDING, state);
}

void test_sm_LANDING_targets_the_ground_without_moving(void) {
    go_to(LANDING);
    cmd_vel_ret.linear.x = 1.0;
    cmd_vel_fresh_ret = true;
    state_machine();
    ATTITUDE_TARGET t = get_attitude();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, t.h);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, t.cmd_vel.linear.x);
}

void test_sm_LANDING_stays_and_blinks_the_red_led_while_flying(void) {
    go_to(LANDING);
    for (int i = 0; i < 4; i++) cycle_after_ms(BLINK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(LANDING, state);
    TEST_ASSERT_EQUAL_INT(2, led_on_count[LED_RED]);
    TEST_ASSERT_EQUAL_INT(2, led_off_count[LED_RED]);
}

void test_sm_LANDING_on_the_ground_goes_to_DISARMING(void) {
    go_to(LANDING);
    state_machine();              /* first cycle: new target 0 m */
    h_reached_ret = true;         /* the controller reached the ground */
    state_machine();
    TEST_ASSERT_EQUAL_INT(DISARMING, state);
}

/* Same as in TAKING_OFF: in HOVERING check_h_reached() is true (hover
 * height reached). HOVERING calls reset_h_reached() before LANDING, so the
 * first cycle of LANDING does not go to DISARMING and stop the motors in
 * the air. */
void test_sm_LANDING_does_not_trust_the_height_of_the_previous_target(void) {
    go_to(HOVERING);
    landing_ready_ret = true;
    h_reached_ret = true;   /* true from HOVERING */
    state_machine();        /* HOVERING -> LANDING */
    state_machine();        /* first cycle of LANDING */
    TEST_ASSERT_EQUAL_INT_MESSAGE(LANDING, state, "the motors are stopped in the air");
}

void test_sm_DISARMING_stops_and_disarms_the_motors(void) {
    go_to(DISARMING);
    state_machine();
    TEST_ASSERT_EQUAL_INT(1, called("motors_stop_all"));
    TEST_ASSERT_EQUAL_INT(1, called("disarm_motors"));
}

void test_sm_DISARMING_goes_back_to_ARMING_with_green_led(void) {
    go_to(DISARMING);
    state_machine();
    TEST_ASSERT_EQUAL_INT(ARMING, state);
    TEST_ASSERT_EQUAL_INT(1, led_on_count[LED_GREEN]);
    TEST_ASSERT_EQUAL_INT(1, led_off_count[LED_RED]);
    TEST_ASSERT_EQUAL_INT(1, led_off_count[LED_BLUE]);
}

/* reset_h_reached(): every transition that changes the target height clears
 * the flag of the attitude controller, so the next state does not use a
 * check_h_reached() computed with the previous target. */
void test_sm_ARMING_without_takeoff_does_not_reset_h_reached(void) {
    go_to(ARMING);
    state_machine();
    TEST_ASSERT_EQUAL_INT(0, called("reset_h_reached"));
}

void test_sm_ARMING_takeoff_resets_h_reached_after_arming(void) {
    go_to(ARMING);
    take_off_ready_ret = true;
    state_machine();
    TEST_ASSERT_EQUAL_INT(1, called("reset_h_reached"));
    TEST_ASSERT_TRUE(call_index("arm_motors") < call_index("reset_h_reached"));
    TEST_ASSERT_EQUAL_INT(TAKING_OFF, state);
}

void test_sm_TAKING_OFF_reached_resets_h_reached(void) {
    go_to(TAKING_OFF);
    h_reached_ret = true;
    state_machine();
    TEST_ASSERT_EQUAL_INT(HOVERING, state);
    TEST_ASSERT_EQUAL_INT(1, called("reset_h_reached"));
}

void test_sm_TAKING_OFF_not_reached_does_not_reset_h_reached(void) {
    go_to(TAKING_OFF);
    for (int i = 0; i < 4; i++) state_machine();
    TEST_ASSERT_EQUAL_INT(0, called("reset_h_reached"));
}

void test_sm_HOVERING_does_not_reset_h_reached_while_hovering(void) {
    go_to(HOVERING);
    for (int i = 0; i < 4; i++) state_machine();
    TEST_ASSERT_EQUAL_INT(0, called("reset_h_reached"));
}

void test_sm_HOVERING_land_resets_h_reached(void) {
    go_to(HOVERING);
    landing_ready_ret = true;
    state_machine();
    TEST_ASSERT_EQUAL_INT(LANDING, state);
    TEST_ASSERT_EQUAL_INT(1, called("reset_h_reached"));
}

void test_sm_EXTERNAL_CONTROL_land_resets_h_reached(void) {
    go_to(EXTERNAL_CONTROL);
    cmd_vel_fresh_ret = true;
    landing_ready_ret = true;
    state_machine();
    TEST_ASSERT_EQUAL_INT(LANDING, state);
    TEST_ASSERT_EQUAL_INT(1, called("reset_h_reached"));
}

void test_sm_LANDING_on_the_ground_resets_h_reached(void) {
    go_to(LANDING);
    h_reached_ret = true;
    state_machine();
    TEST_ASSERT_EQUAL_INT(DISARMING, state);
    TEST_ASSERT_EQUAL_INT(1, called("reset_h_reached"));
}

void test_sm_full_flight_resets_h_reached_in_every_height_change(void) {
    go_to(ARMING);
    take_off_ready_ret = true;
    state_machine();                 /* ARMING -> TAKING_OFF */
    take_off_ready_ret = false;
    h_reached_ret = true;
    state_machine();                 /* TAKING_OFF -> HOVERING */
    landing_ready_ret = true;
    state_machine();                 /* HOVERING -> LANDING */
    h_reached_ret = true;
    state_machine();                 /* LANDING -> DISARMING */
    TEST_ASSERT_EQUAL_INT(DISARMING, state);
    TEST_ASSERT_EQUAL_INT(4, called("reset_h_reached"));
}

void test_sm_ERROR_is_final(void) {
    go_to(ERROR);
    take_off_ready_ret = true;
    for (int i = 0; i < 20; i++) state_machine();
    TEST_ASSERT_EQUAL_INT(ERROR, state);
    TEST_ASSERT_EQUAL_INT(0, called("arm_motors"));
}

void test_sm_full_flight(void) {
    state_machine();                                 /* INIT */
    TEST_ASSERT_EQUAL_INT(CHECKING, global_sm());
    state_machine();                                 /* CHECKING */
    TEST_ASSERT_EQUAL_INT(ARMING, global_sm());
    state_machine();                                 /* ARMING, no takeoff */
    TEST_ASSERT_EQUAL_INT(ARMING, global_sm());
    take_off_ready_ret = true;
    target_altitude_ret = 1.2f;
    state_machine();                                 /* ARMING, takeoff */
    TEST_ASSERT_EQUAL_INT(TAKING_OFF, global_sm());
    state_machine();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.2f, get_attitude().h);
    h_reached_ret = true;
    state_machine();                                 /* TAKING_OFF, at 1.2 m */
    TEST_ASSERT_EQUAL_INT(HOVERING, global_sm());
    cmd_vel_fresh_ret = true;
    state_machine();                                 /* HOVERING, cmd_vel */
    TEST_ASSERT_EQUAL_INT(EXTERNAL_CONTROL, global_sm());
    cmd_vel_fresh_ret = false;
    state_machine();                                 /* EXTERNAL_CONTROL, old cmd_vel */
    TEST_ASSERT_EQUAL_INT(HOVERING, global_sm());
    landing_ready_ret = true;
    state_machine();                                 /* HOVERING, land */
    TEST_ASSERT_EQUAL_INT(LANDING, global_sm());
    h_reached_ret = false;
    state_machine();                                 /* LANDING, going down */
    TEST_ASSERT_EQUAL_INT(LANDING, global_sm());
    h_reached_ret = true;
    state_machine();                                 /* LANDING, on the ground */
    TEST_ASSERT_EQUAL_INT(DISARMING, global_sm());
    state_machine();                                 /* DISARMING */
    TEST_ASSERT_EQUAL_INT(ARMING, global_sm());
    TEST_ASSERT_EQUAL_INT(1, called("disarm_motors"));
}

/* ------------------------------------------------------------------ */
/*                                TASK                                */
/* ------------------------------------------------------------------ */
void test_task_start_creates_the_task_with_the_menuconfig_values(void) {
    system_start();
    const mock_task_t *t = mock_find_task("system_task");
    TEST_ASSERT_NOT_NULL(t);
    TEST_ASSERT_EQUAL_UINT32(CONFIG_SYSTEM_TASK_STACK, t->stack);
    TEST_ASSERT_EQUAL_UINT(CONFIG_SYSTEM_TASK_PRIO, t->prio);
}

void test_task_start_does_not_init_by_itself(void) {
    /* the init is done by the task, in the INIT state */
    system_start();
    TEST_ASSERT_FALSE(is_init);
}

void test_task_runs_the_state_machine_every_20_ms(void) {
    TEST_ASSERT_EQUAL_INT(20, SYSTEM_TASK_PERIOD_MS);
    go_to(ERROR);   /* a state without its own delays */
    mock_run_task(system_task, NULL, 10);
    TEST_ASSERT_EQUAL_UINT64(10 * SYSTEM_TASK_PERIOD_MS, mock_delay_total_ms);
}

void test_task_from_INIT_reaches_ARMING_and_waits(void) {
    /* system_init() also waits (500 ms between modules): give enough delays */
    mock_run_task(system_task, NULL, 50);
    TEST_ASSERT_EQUAL_INT(ARMING, global_sm());
    TEST_ASSERT_EQUAL_INT(0, called("arm_motors"));
}

/* ------------------------------------------------------------------ */
/*                              FAILSAFE                              */
/* ------------------------------------------------------------------ */
/* Advances in cycles of 20 ms, like the task. */
static void run_ms(int ms) {
    for (int t = 0; t < ms; t += SYSTEM_TASK_PERIOD_MS) {
        cycle_after_ms(SYSTEM_TASK_PERIOD_MS);
    }
}

void test_failsafe_sensors_with_data_do_not_start_it(void) {
    go_to(HOVERING);
    run_ms(2000);
    TEST_ASSERT_EQUAL_INT(HOVERING, state);
}

void test_failsafe_is_not_used_on_the_ground(void) {
    go_to(ARMING);
    imu_alive = baro_alive = false;
    run_ms(2000);
    TEST_ASSERT_EQUAL_INT(ARMING, state);
}

void test_failsafe_take_off_without_recent_samples_starts_it(void) {
    /* Same as no sample: the last one is older than the timeouts */
    go_to(TAKING_OFF);
    mock_time_us += BARO_TIMEOUT_US + 1;
    state_machine();
    TEST_ASSERT_EQUAL_INT(EMERGENCY_LANDING, state);
}

void test_failsafe_baro_waits_for_its_timeout(void) {
    go_to(HOVERING);
    baro_alive = false;
    run_ms(BARO_TIMEOUT_US / 1000 - SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(HOVERING, state);
    run_ms(2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(EMERGENCY_LANDING, state);
    TEST_ASSERT_TRUE(get_barometer_failing());
}

/* h = 2 m, a = 0.5 m/s^2 -> sqrt(2 * 2 / 0.5) + 1 s of margin = 3.83 s */
void test_failsafe_baro_stops_the_motors_only_when_the_time_elapses(void) {
    h_est_ret = 2.0f;
    go_to(HOVERING);
    baro_alive = false;
    run_ms(BARO_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(EMERGENCY_LANDING, state);

    run_ms(3700);
    TEST_ASSERT_EQUAL_INT_MESSAGE(EMERGENCY_LANDING, state, "the motors are stopped in the air");
    TEST_ASSERT_EQUAL_INT(0, called("disarm_motors"));

    run_ms(300);
    TEST_ASSERT_EQUAL_INT(ERROR, state);
    TEST_ASSERT_EQUAL_INT(1, called("disarm_motors"));
    TEST_ASSERT_EQUAL_INT(1, called("motors_stop_all"));
    TEST_ASSERT_TRUE(call_index("disarm_motors") < call_index("motors_stop_all"));
}

void test_failsafe_does_not_trust_a_negative_height(void) {
    h_est_ret = -3.0f;
    go_to(HOVERING);
    baro_alive = false;
    run_ms(BARO_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT64(1000000, emergency_time_us);
}

void test_failsafe_ignores_cmd_vel_while_landing(void) {
    go_to(EXTERNAL_CONTROL);
    cmd_vel_ret.linear.x = 1.0;
    cmd_vel_fresh_ret = true;
    state_machine();
    TEST_ASSERT_EQUAL_DOUBLE(1.0, get_attitude().cmd_vel.linear.x);

    baro_alive = false;
    run_ms(BARO_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(EMERGENCY_LANDING, state);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, get_attitude().cmd_vel.linear.x);
}

void test_failsafe_imu_ramps_the_same_power_to_0_and_ends_in_ERROR(void) {
    last_motor_power_ret = 50.0f;
    go_to(HOVERING);
    imu_alive = false;
    run_ms(IMU_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(EMERGENCY_LANDING, state);
    TEST_ASSERT_FALSE(get_barometer_failing());

    double prev = motor_power_ret[0];
    TEST_ASSERT_TRUE(prev > 0.0);
    while (state == EMERGENCY_LANDING) {
        TEST_ASSERT_TRUE(mock_time_us - t_imu_cut_us < 2 * IMU_CUT_TIME_MS * 1000LL);
        cycle_after_ms(SYSTEM_TASK_PERIOD_MS);
        TEST_ASSERT_TRUE(motor_power_ret[0] <= prev);
        for (int x = 1; x < N_MOTORS; x++) {
            TEST_ASSERT_EQUAL_DOUBLE(motor_power_ret[0], motor_power_ret[x]);
        }
        prev = motor_power_ret[0];
    }
    TEST_ASSERT_EQUAL_INT(ERROR, state);
    TEST_ASSERT_EQUAL_INT(1, called("disarm_motors"));
    TEST_ASSERT_EQUAL_INT(1, called("motors_stop_all"));
}

void test_failsafe_imu_failing_while_landing_without_baro_escalates(void) {
    h_est_ret = 10.0f; /* Long landing */
    last_motor_power_ret = 40.0f;
    go_to(HOVERING);
    baro_alive = false;
    run_ms(BARO_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_TRUE(get_barometer_failing());

    imu_alive = false;
    run_ms(IMU_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(IMU_FAILURE, f_type);
    run_ms(IMU_CUT_TIME_MS + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(ERROR, state);
}

void test_failsafe_is_not_cancelled_if_the_sensor_comes_back(void) {
    go_to(HOVERING);
    baro_alive = false;
    run_ms(BARO_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(EMERGENCY_LANDING, state);
    baro_alive = true;
    run_ms(100);
    TEST_ASSERT_EQUAL_INT(EMERGENCY_LANDING, state);
    TEST_ASSERT_TRUE(get_barometer_failing());
}

void test_failsafe_ERROR_is_final_after_it(void) {
    go_to(HOVERING);
    baro_alive = false;
    run_ms(BARO_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS + 1100);
    TEST_ASSERT_EQUAL_INT(ERROR, state);
    run_ms(1000);
    TEST_ASSERT_EQUAL_INT(ERROR, state);
}

#ifdef CONFIG_SIMULATION_ON
/* In simulation only the IMU topic can stop while the link is ok: the
 * motor speeds still arrive, so it must ramp them down, not stop them. */
void test_failsafe_sim_imu_ramps_and_does_not_stop_at_once(void) {
    last_motor_power_ret = 50.0f;
    go_to(HOVERING);
    imu_alive = false;
    run_ms(IMU_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(EMERGENCY_LANDING, state);
    TEST_ASSERT_EQUAL_INT(0, called("motors_stop_all"));
    TEST_ASSERT_TRUE(called("set_motor_speed") > 0);
    TEST_ASSERT_TRUE(motor_power_ret[0] > 0.0);
}

/* No ping in simulation: the link is not checked. */
void test_failsafe_sim_does_not_check_the_link(void) {
    go_to(HOVERING);
    link_alive = false;
    run_ms(LINK_TIMEOUT_US / 1000 + 1000);
    TEST_ASSERT_EQUAL_INT(HOVERING, state);
    TEST_ASSERT_EQUAL_INT(0, called("get_t_last_link"));
}
#else
void test_failsafe_link_waits_for_its_timeout(void) {
    go_to(HOVERING);
    link_alive = false;
    run_ms(LINK_TIMEOUT_US / 1000 - SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(HOVERING, state);
    run_ms(2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(EMERGENCY_LANDING, state);
    TEST_ASSERT_EQUAL_INT(LINK_FAILURE, f_type);
    TEST_ASSERT_FALSE(get_barometer_failing());
    TEST_ASSERT_FALSE(get_imu_failing());
}

void test_failsafe_link_is_not_used_on_the_ground(void) {
    go_to(ARMING);
    link_alive = false;
    run_ms(LINK_TIMEOUT_US / 1000 + 1000);
    TEST_ASSERT_EQUAL_INT(ARMING, state);
}

/* Lands with the sensors (h = 0, no velocity) and ignores ROS. */
void test_failsafe_link_lands_to_0_and_ignores_cmd_vel(void) {
    go_to(EXTERNAL_CONTROL);
    cmd_vel_ret.linear.x = 1.0;
    cmd_vel_fresh_ret = true;
    link_alive = false;
    run_ms(LINK_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(EMERGENCY_LANDING, state);
    clear_logs();
    run_ms(200);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, get_attitude().h);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, get_attitude().cmd_vel.linear.x);
    TEST_ASSERT_EQUAL_INT(0, called("set_motor_speed")); /* The controller flies */

    landing_ready_ret = true; /* A land order can not arrive, but it is ignored */
    run_ms(200);
    TEST_ASSERT_EQUAL_INT(EMERGENCY_LANDING, state);
}

void test_failsafe_link_on_the_ground_goes_to_ERROR(void) {
    h_est_ret = 1.0f;
    go_to(HOVERING);
    link_alive = false;
    run_ms(LINK_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(EMERGENCY_LANDING, state);
    TEST_ASSERT_EQUAL_INT(0, called("disarm_motors"));

    clear_logs(); /* The log is full after the timeout */
    h_reached_ret = true; /* The controller says that it is at h = 0 */
    run_ms(2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(ERROR, state);
    TEST_ASSERT_EQUAL_INT(1, called("disarm_motors"));
    TEST_ASSERT_EQUAL_INT(1, called("motors_stop_all"));
    TEST_ASSERT_TRUE(call_index("disarm_motors") < call_index("motors_stop_all"));
}

/* h = 1.5 m at 0.15 m/s -> 10 s + 3 s of margin = 13 s */
void test_failsafe_link_stops_the_motors_if_the_ground_is_not_detected(void) {
    h_est_ret = 1.5f;
    go_to(HOVERING);
    link_alive = false;
    run_ms(LINK_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT64(13000000, emergency_time_us);

    run_ms(12800);
    TEST_ASSERT_EQUAL_INT(EMERGENCY_LANDING, state);
    run_ms(300);
    TEST_ASSERT_EQUAL_INT(ERROR, state);
}

/* A worse failure while landing without link starts its own emergency. */
void test_failsafe_link_landing_escalates_to_baro(void) {
    h_est_ret = 2.0f;
    go_to(HOVERING);
    link_alive = false;
    run_ms(LINK_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(LINK_FAILURE, f_type);

    baro_alive = false;
    run_ms(BARO_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_TRUE(get_barometer_failing());
    TEST_ASSERT_EQUAL_INT(BARO_FAILURE, f_emergency);
    /* Time of the landing without barometer: sqrt(2 * 2 / 0.5) + 1 s */
    TEST_ASSERT_INT64_WITHIN(1000, 3828427, emergency_time_us);
}

void test_failsafe_link_landing_escalates_to_imu(void) {
    last_motor_power_ret = 40.0f;
    go_to(HOVERING);
    link_alive = false;
    run_ms(LINK_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    clear_logs(); /* The log is full after the timeout */
    imu_alive = false;
    run_ms(IMU_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_TRUE(get_imu_failing());
    TEST_ASSERT_TRUE(called("set_motor_speed") > 0); /* The power ramp */
    run_ms(IMU_CUT_TIME_MS + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(ERROR, state);
}

/* The link is the least bad failure: it does not replace the barometer. */
void test_failsafe_link_does_not_replace_a_baro_failure(void) {
    h_est_ret = 10.0f;
    go_to(HOVERING);
    baro_alive = false;
    run_ms(BARO_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    link_alive = false;
    run_ms(LINK_TIMEOUT_US / 1000 + 2 * SYSTEM_TASK_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT(BARO_FAILURE, f_type);
}
#endif

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_init_starts_every_module);
    RUN_TEST(test_init_micro_ros_is_the_last_one);
    RUN_TEST(test_init_sensors_before_the_controller);
    RUN_TEST(test_init_resets_the_global_state);
    RUN_TEST(test_init_twice_starts_the_modules_once);
    RUN_TEST(test_init_odometry_after_the_controller);

    RUN_TEST(test_selftest_false_before_init);
    RUN_TEST(test_selftest_true_when_everything_is_ok);
    RUN_TEST(test_selftest_fails_if_the_leds_fail);
    RUN_TEST(test_selftest_fails_if_micro_ros_fails);
#ifdef CONFIG_SIMULATION_ON
    RUN_TEST(test_selftest_simulation_does_not_test_the_sensors);
#else
    RUN_TEST(test_selftest_fails_if_the_imu_fails);
    RUN_TEST(test_selftest_fails_if_the_barometer_fails);
#endif
    RUN_TEST(test_selftest_runs_the_motor_test);
    RUN_TEST(test_selftest_fails_if_the_motors_fail);

    RUN_TEST(test_sm_change_state_updates_the_global_state);
    RUN_TEST(test_sm_starts_in_INIT);
    RUN_TEST(test_sm_INIT_starts_the_system_and_goes_to_CHECKING);
    RUN_TEST(test_sm_CHECKING_ok_goes_to_ARMING_with_green_led);
    RUN_TEST(test_sm_CHECKING_fail_goes_to_ERROR);
    RUN_TEST(test_sm_ARMING_waits_for_the_takeoff);
    RUN_TEST(test_sm_ARMING_does_not_wait_inside_the_state);
    RUN_TEST(test_sm_ARMING_with_takeoff_arms_and_goes_to_TAKING_OFF);
    RUN_TEST(test_sm_TAKING_OFF_targets_the_takeoff_altitude_without_moving);
    RUN_TEST(test_sm_TAKING_OFF_stays_while_the_height_is_not_reached);
    RUN_TEST(test_sm_TAKING_OFF_blinks_the_blue_led);
    RUN_TEST(test_sm_blink_changes_the_led_every_blink_period);
    RUN_TEST(test_sm_EXTERNAL_CONTROL_blinks_the_green_led_without_waiting);
    RUN_TEST(test_sm_TAKING_OFF_height_reached_goes_to_HOVERING_with_blue_led);
    RUN_TEST(test_sm_TAKING_OFF_does_not_trust_the_height_of_the_previous_target);
    RUN_TEST(test_sm_HOVERING_keeps_the_altitude_and_does_not_move);
    RUN_TEST(test_sm_HOVERING_new_cmd_vel_goes_to_EXTERNAL_CONTROL);
    RUN_TEST(test_sm_HOVERING_land_goes_to_LANDING);
    RUN_TEST(test_sm_HOVERING_land_has_priority_over_cmd_vel);
    RUN_TEST(test_sm_EXTERNAL_CONTROL_follows_cmd_vel);
    RUN_TEST(test_sm_EXTERNAL_CONTROL_uses_the_cmd_vel_expire_time);
    RUN_TEST(test_sm_EXTERNAL_CONTROL_old_cmd_vel_goes_to_HOVERING);
    RUN_TEST(test_sm_EXTERNAL_CONTROL_land_goes_to_LANDING);
    RUN_TEST(test_sm_LANDING_targets_the_ground_without_moving);
    RUN_TEST(test_sm_LANDING_stays_and_blinks_the_red_led_while_flying);
    RUN_TEST(test_sm_LANDING_on_the_ground_goes_to_DISARMING);
    RUN_TEST(test_sm_LANDING_does_not_trust_the_height_of_the_previous_target);
    RUN_TEST(test_sm_DISARMING_stops_and_disarms_the_motors);
    RUN_TEST(test_sm_DISARMING_goes_back_to_ARMING_with_green_led);
    RUN_TEST(test_sm_ARMING_without_takeoff_does_not_reset_h_reached);
    RUN_TEST(test_sm_ARMING_takeoff_resets_h_reached_after_arming);
    RUN_TEST(test_sm_TAKING_OFF_reached_resets_h_reached);
    RUN_TEST(test_sm_TAKING_OFF_not_reached_does_not_reset_h_reached);
    RUN_TEST(test_sm_HOVERING_does_not_reset_h_reached_while_hovering);
    RUN_TEST(test_sm_HOVERING_land_resets_h_reached);
    RUN_TEST(test_sm_EXTERNAL_CONTROL_land_resets_h_reached);
    RUN_TEST(test_sm_LANDING_on_the_ground_resets_h_reached);
    RUN_TEST(test_sm_full_flight_resets_h_reached_in_every_height_change);
    RUN_TEST(test_sm_ERROR_is_final);
    RUN_TEST(test_sm_full_flight);

    RUN_TEST(test_failsafe_sensors_with_data_do_not_start_it);
    RUN_TEST(test_failsafe_is_not_used_on_the_ground);
    RUN_TEST(test_failsafe_take_off_without_recent_samples_starts_it);
    RUN_TEST(test_failsafe_baro_waits_for_its_timeout);
    RUN_TEST(test_failsafe_baro_stops_the_motors_only_when_the_time_elapses);
    RUN_TEST(test_failsafe_does_not_trust_a_negative_height);
    RUN_TEST(test_failsafe_ignores_cmd_vel_while_landing);
    RUN_TEST(test_failsafe_imu_ramps_the_same_power_to_0_and_ends_in_ERROR);
    RUN_TEST(test_failsafe_imu_failing_while_landing_without_baro_escalates);
    RUN_TEST(test_failsafe_is_not_cancelled_if_the_sensor_comes_back);
    RUN_TEST(test_failsafe_ERROR_is_final_after_it);
#ifdef CONFIG_SIMULATION_ON
    RUN_TEST(test_failsafe_sim_imu_ramps_and_does_not_stop_at_once);
    RUN_TEST(test_failsafe_sim_does_not_check_the_link);
#else
    RUN_TEST(test_failsafe_link_waits_for_its_timeout);
    RUN_TEST(test_failsafe_link_is_not_used_on_the_ground);
    RUN_TEST(test_failsafe_link_lands_to_0_and_ignores_cmd_vel);
    RUN_TEST(test_failsafe_link_on_the_ground_goes_to_ERROR);
    RUN_TEST(test_failsafe_link_stops_the_motors_if_the_ground_is_not_detected);
    RUN_TEST(test_failsafe_link_landing_escalates_to_baro);
    RUN_TEST(test_failsafe_link_landing_escalates_to_imu);
    RUN_TEST(test_failsafe_link_does_not_replace_a_baro_failure);
#endif

    RUN_TEST(test_task_start_creates_the_task_with_the_menuconfig_values);
    RUN_TEST(test_task_start_does_not_init_by_itself);
    RUN_TEST(test_task_runs_the_state_machine_every_20_ms);
    RUN_TEST(test_task_from_INIT_reaches_ARMING_and_waits);

    return UNITY_END();
}
