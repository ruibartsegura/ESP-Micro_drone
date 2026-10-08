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
 *   Transitions that are still TODO in system.c are IGNORED tests: remove
 *   the TEST_IGNORE_MESSAGE() line when you implement them.
 *
 *   The .c file is included directly. Another version can be tested with:
 *       make run T=system_hw MODULE_SRC=path/to/system.c
 *
 *   Tests that fail with the current code have [BUG] in their comment.
 *
 * Functions:
 *   - Mocks of led, imu, height, motors, attitude_controller, ros_coordinator.
 *   - called() / call_index(): look in the log of calls.
 *   - go_to(): puts the state machine in a state.
 *   - setUp() / tearDown(): reset the mocks, the module and the state.
 *   - test_init_*, test_selftest_*, test_sm_*, test_takeoff_check_*, test_task_*.
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
static bool take_off_ready_ret;
static float take_off_alt_ret;
static geometry_msgs__msg__Twist cmd_vel_ret;
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
void motors_init(void)     { log_call("motors_init"); }
bool motors_test(void)     { log_call("motors_test"); return motors_test_ret; }
void arm_motors(void)      { log_call("arm_motors"); }
void disarm_motors(void)   { log_call("disarm_motors"); }
void motors_stop_all(void) { log_call("motors_stop_all"); }
void init_attitude_controller(void) { log_call("init_attitude_controller"); }
void init_tuning(void)     { log_call("init_tuning"); }
void ros_init(void)        { log_call("ros_init"); }
bool ros_test(void)        { log_call("ros_test"); return ros_test_ret; }
bool get_take_off_ready(void) { log_call("get_take_off_ready"); return take_off_ready_ret; }
float get_take_off_alt(void)  { log_call("get_take_off_alt"); return take_off_alt_ret; }
geometry_msgs__msg__Twist get_cmd_vel(void) { log_call("get_cmd_vel"); return cmd_vel_ret; }
void odom_estimator_init(void) { log_call("odom_estimator_init"); }

/* ------------------------------------------------------------------ */
/*                              HELPERS                               */
/* ------------------------------------------------------------------ */
static sm_states_t global_sm(void) {
    sm_states_t s;
    get_sm_state(&s);
    return s;
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
    take_off_ready_ret = false;
    take_off_alt_ret = 1.0f;
    memset(&cmd_vel_ret, 0, sizeof cmd_vel_ret);
}

void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                              INIT                                  */
/* ------------------------------------------------------------------ */
void test_init_starts_every_module(void) {
    system_init();
    TEST_ASSERT_EQUAL_INT(1, called("led_init"));
    TEST_ASSERT_EQUAL_INT(1, called("motors_init"));
    TEST_ASSERT_EQUAL_INT(1, called("init_attitude_controller"));
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
    TEST_ASSERT_TRUE(call_index("motors_init") < call_index("init_attitude_controller"));
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

/* [BUG] odom_estimator_init() is never called, so the odometry task
 * (odom_task, 100 Hz) described in the documentation never runs. */
void test_init_starts_the_odometry(void) {
    system_init();
    TEST_ASSERT_EQUAL_INT(1, called("odom_estimator_init"));
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

void test_selftest_does_not_spin_the_motors(void) {
    /* the motor test is disabled: it must not run with the propellers on */
    system_init();
    system_test();
    TEST_ASSERT_EQUAL_INT(0, called("motors_test"));
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

void test_sm_ARMING_checks_the_service_every_150_ms(void) {
    go_to(ARMING);
    state_machine();
    TEST_ASSERT_EQUAL_UINT64(150, mock_delay_total_ms);
}

void test_sm_ARMING_with_takeoff_arms_and_goes_to_TAKING_OFF(void) {
    go_to(ARMING);
    take_off_ready_ret = true;
    state_machine();
    TEST_ASSERT_EQUAL_INT(1, called("arm_motors"));
    TEST_ASSERT_EQUAL_INT(TAKING_OFF, state);
}

/* [BUG] The blue LED should blink 3 times when the drone is armed, but
 * the loop only calls led_on(): the LED stays on, it never blinks. */
void test_sm_ARMING_blinks_the_blue_led_3_times(void) {
    go_to(ARMING);
    take_off_ready_ret = true;
    state_machine();
    TEST_ASSERT_EQUAL_INT(3, led_on_count[LED_BLUE]);
    TEST_ASSERT_EQUAL_INT_MESSAGE(3, led_off_count[LED_BLUE], "to blink, the LED must also be turned off");
}

void test_sm_TAKING_OFF_targets_the_takeoff_altitude_without_moving(void) {
    go_to(TAKING_OFF);
    take_off_alt_ret = 1.5f;
    cmd_vel_ret.linear.x = 3.0;   /* must be ignored */
    state_machine();
    ATTITUDE_TARGET t = get_attitude();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.5f, t.h);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, t.cmd_vel.linear.x);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, t.cmd_vel.angular.z);
}

void test_sm_TAKING_OFF_stays_while_below_the_altitude(void) {
    go_to(TAKING_OFF);
    take_off_alt_ret = 1.0f;
    set_h(0.3f);
    state_machine();
    TEST_ASSERT_EQUAL_INT(TAKING_OFF, state);
}

/* [BUG] check_takeOff_2_hov() uses "float h = 0; //get_h();" instead of the
 * real height, so the drone never goes from TAKING_OFF to HOVERING. */
void test_sm_TAKING_OFF_at_the_altitude_goes_to_HOVERING(void) {
    go_to(TAKING_OFF);
    take_off_alt_ret = 1.0f;
    set_h(1.0f);
    state_machine();
    TEST_ASSERT_EQUAL_INT(HOVERING, state);
}

void test_sm_HOVERING_keeps_the_altitude_and_does_not_move(void) {
    go_to(HOVERING);
    take_off_alt_ret = 0.8f;
    cmd_vel_ret.linear.y = 1.0;
    state_machine();
    ATTITUDE_TARGET t = get_attitude();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.8f, t.h);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, t.cmd_vel.linear.y);
    TEST_ASSERT_EQUAL_INT(HOVERING, state);
}

void test_sm_EXTERNAL_CONTROL_follows_cmd_vel(void) {
    go_to(EXTERNAL_CONTROL);
    take_off_alt_ret = 0.8f;
    cmd_vel_ret.linear.x = 0.5;
    cmd_vel_ret.angular.z = 0.2;
    state_machine();
    ATTITUDE_TARGET t = get_attitude();
    TEST_ASSERT_EQUAL_DOUBLE(0.5, t.cmd_vel.linear.x);
    TEST_ASSERT_EQUAL_DOUBLE(0.2, t.cmd_vel.angular.z);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.8f, t.h);
}

void test_sm_LANDING_targets_the_ground_without_moving(void) {
    go_to(LANDING);
    cmd_vel_ret.linear.x = 1.0;
    state_machine();
    ATTITUDE_TARGET t = get_attitude();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, t.h);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, t.cmd_vel.linear.x);
}

void test_sm_DISARMING_stops_and_disarms_the_motors(void) {
    go_to(DISARMING);
    state_machine();
    TEST_ASSERT_EQUAL_INT(1, called("motors_stop_all"));
    TEST_ASSERT_EQUAL_INT(1, called("disarm_motors"));
    TEST_ASSERT_TRUE(call_index("motors_stop_all") < call_index("disarm_motors"));
}

void test_sm_ERROR_is_final(void) {
    go_to(ERROR);
    take_off_ready_ret = true;
    for (int i = 0; i < 20; i++) state_machine();
    TEST_ASSERT_EQUAL_INT(ERROR, state);
    TEST_ASSERT_EQUAL_INT(0, called("arm_motors"));
}

void test_sm_full_flight_until_TAKING_OFF(void) {
    state_machine();                                 /* INIT */
    TEST_ASSERT_EQUAL_INT(CHECKING, global_sm());
    state_machine();                                 /* CHECKING */
    TEST_ASSERT_EQUAL_INT(ARMING, global_sm());
    state_machine();                                 /* ARMING, no takeoff */
    TEST_ASSERT_EQUAL_INT(ARMING, global_sm());
    take_off_ready_ret = true;
    take_off_alt_ret = 1.2f;
    state_machine();                                 /* ARMING, takeoff */
    TEST_ASSERT_EQUAL_INT(TAKING_OFF, global_sm());
    state_machine();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.2f, get_attitude().h);
}

void test_sm_HOVERING_new_cmd_vel_goes_to_EXTERNAL_CONTROL(void) {
    TEST_IGNORE_MESSAGE("TODO in system.c: HOVERING -> EXTERNAL_CONTROL is not implemented");
}

void test_sm_HOVERING_land_goes_to_LANDING(void) {
    TEST_IGNORE_MESSAGE("TODO in system.c: HOVERING -> LANDING is not implemented");
}

void test_sm_EXTERNAL_CONTROL_old_cmd_vel_goes_to_HOVERING(void) {
    TEST_IGNORE_MESSAGE("TODO in system.c: EXTERNAL_CONTROL -> HOVERING (old cmd_vel) is not implemented");
}

void test_sm_LANDING_on_the_ground_goes_to_DISARMING(void) {
    TEST_IGNORE_MESSAGE("TODO in system.c: LANDING -> DISARMING is not implemented");
}

/* ------------------------------------------------------------------ */
/*                       TAKE-OFF ALTITUDE CHECK                      */
/* ------------------------------------------------------------------ */
/* [BUG] Same bug as above: the height is always 0. */
void test_takeoff_check_true_at_the_altitude(void) {
    set_h(1.0f);
    TEST_ASSERT_TRUE(check_takeOff_2_hov(1.0f));
}

/* [BUG] Same bug: the margin is +-5 %. */
void test_takeoff_check_true_inside_5_percent(void) {
    set_h(0.96f);
    TEST_ASSERT_TRUE(check_takeOff_2_hov(1.0f));
    set_h(1.04f);
    TEST_ASSERT_TRUE(check_takeOff_2_hov(1.0f));
}

void test_takeoff_check_false_outside_5_percent(void) {
    set_h(0.90f);
    TEST_ASSERT_FALSE(check_takeOff_2_hov(1.0f));
    set_h(1.10f);
    TEST_ASSERT_FALSE(check_takeOff_2_hov(1.0f));
}

void test_takeoff_check_false_on_the_ground(void) {
    set_h(0.0f);
    TEST_ASSERT_FALSE(check_takeOff_2_hov(1.0f));
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

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_init_starts_every_module);
    RUN_TEST(test_init_micro_ros_is_the_last_one);
    RUN_TEST(test_init_sensors_before_the_controller);
    RUN_TEST(test_init_resets_the_global_state);
    RUN_TEST(test_init_twice_starts_the_modules_once);
    RUN_TEST(test_init_starts_the_odometry);

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
    RUN_TEST(test_selftest_does_not_spin_the_motors);

    RUN_TEST(test_sm_change_state_updates_the_global_state);
    RUN_TEST(test_sm_starts_in_INIT);
    RUN_TEST(test_sm_INIT_starts_the_system_and_goes_to_CHECKING);
    RUN_TEST(test_sm_CHECKING_ok_goes_to_ARMING_with_green_led);
    RUN_TEST(test_sm_CHECKING_fail_goes_to_ERROR);
    RUN_TEST(test_sm_ARMING_waits_for_the_takeoff);
    RUN_TEST(test_sm_ARMING_checks_the_service_every_150_ms);
    RUN_TEST(test_sm_ARMING_with_takeoff_arms_and_goes_to_TAKING_OFF);
    RUN_TEST(test_sm_ARMING_blinks_the_blue_led_3_times);
    RUN_TEST(test_sm_TAKING_OFF_targets_the_takeoff_altitude_without_moving);
    RUN_TEST(test_sm_TAKING_OFF_stays_while_below_the_altitude);
    RUN_TEST(test_sm_TAKING_OFF_at_the_altitude_goes_to_HOVERING);
    RUN_TEST(test_sm_HOVERING_keeps_the_altitude_and_does_not_move);
    RUN_TEST(test_sm_EXTERNAL_CONTROL_follows_cmd_vel);
    RUN_TEST(test_sm_LANDING_targets_the_ground_without_moving);
    RUN_TEST(test_sm_DISARMING_stops_and_disarms_the_motors);
    RUN_TEST(test_sm_ERROR_is_final);
    RUN_TEST(test_sm_full_flight_until_TAKING_OFF);
    RUN_TEST(test_sm_HOVERING_new_cmd_vel_goes_to_EXTERNAL_CONTROL);
    RUN_TEST(test_sm_HOVERING_land_goes_to_LANDING);
    RUN_TEST(test_sm_EXTERNAL_CONTROL_old_cmd_vel_goes_to_HOVERING);
    RUN_TEST(test_sm_LANDING_on_the_ground_goes_to_DISARMING);

    RUN_TEST(test_takeoff_check_true_at_the_altitude);
    RUN_TEST(test_takeoff_check_true_inside_5_percent);
    RUN_TEST(test_takeoff_check_false_outside_5_percent);
    RUN_TEST(test_takeoff_check_false_on_the_ground);

    RUN_TEST(test_task_start_creates_the_task_with_the_menuconfig_values);
    RUN_TEST(test_task_start_does_not_init_by_itself);
    RUN_TEST(test_task_runs_the_state_machine_every_20_ms);
    RUN_TEST(test_task_from_INIT_reaches_ARMING_and_waits);

    return UNITY_END();
}
