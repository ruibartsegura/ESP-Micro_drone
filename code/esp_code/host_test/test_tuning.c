/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for tuning.c (host, no hardware). The real parameters.c is
 *   linked: the names and ranges of the commands are the ones of
 *   param_info[] (the same of the drone/params ROS topic).
 *
 *   The .c file is included directly. Another version can be tested with:
 *       make run T=tuning MODULE_SRC=path/to/tuning.c
 *
 * Functions:
 *   - setUp() / tearDown(): reset the mocks, the module and the parameters.
 *   - test_cmd_*: commands that change or print the gains.
 *   - test_bad_*: invalid commands, the gains must not change.
 *   - test_task_*: the FreeRTOS task.
 *   - main(): runs all the tests.
 */
#include "unity.h"
#include "test_helpers.h"
#include "mocks/mock_reset.h"

#ifndef MODULE_SRC
#define MODULE_SRC "../components/flight_control/tuning.c"
#endif
#include MODULE_SRC

/* Values that each test starts with (different from the defaults, so a
 * "reset" can be seen). */
static const GAINS START = {
    .kp_h = 100.0f, .kd_h = 110.0f, .kp_vel = 0.05f, .kp_angle = 2.0f,
    .kp_rate = 10.0f, .ki_rate = 0.6f, .kp_yaw_rate = 3.0f,
    .max_rate = 200.0f, .max_pow_rp = 400.0f, .base = 2400.0f,
    .alpha = 0.98f, .max_angle = 0.35f, .max_pow_yaw = 150.0f, .pitch_deviation = 10.5f,
};

static GAINS now(void) {
    GAINS k;
    get_gains(&k);
    return k;
}

static void assert_gains_equal(const GAINS *e, const GAINS *a) {
    TEST_ASSERT_EQUAL_MEMORY(e, a, sizeof *e);
}

void setUp(void) {
    mock_reset_all();
    is_init = false;
    params_init();
    set_gains(&START);
}

void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                             COMMANDS                               */
/* ------------------------------------------------------------------ */
void test_cmd_change_one_gain(void) {
    TEST_ASSERT_TRUE(tuning_parse_line("kp_rate 8.5"));
    GAINS e = START;
    e.kp_rate = 8.5f;
    { GAINS a = now(); assert_gains_equal(&e, &a); }
}

void test_cmd_every_gain_name(void) {
    TEST_ASSERT_TRUE(tuning_parse_line("kp_h 150"));
    TEST_ASSERT_TRUE(tuning_parse_line("kd_h 90"));
    TEST_ASSERT_TRUE(tuning_parse_line("kp_vel 0.08"));
    TEST_ASSERT_TRUE(tuning_parse_line("kp_angle 3"));
    TEST_ASSERT_TRUE(tuning_parse_line("kp_rate 7"));
    TEST_ASSERT_TRUE(tuning_parse_line("ki_rate 1"));
    TEST_ASSERT_TRUE(tuning_parse_line("kp_yaw_rate 4"));
    TEST_ASSERT_TRUE(tuning_parse_line("max_rate 150"));
    TEST_ASSERT_TRUE(tuning_parse_line("max_pow_rp 300"));
    TEST_ASSERT_TRUE(tuning_parse_line("base 2390"));
    TEST_ASSERT_TRUE(tuning_parse_line("alpha 0.9"));
    TEST_ASSERT_TRUE(tuning_parse_line("max_angle 0.3"));
    TEST_ASSERT_TRUE(tuning_parse_line("max_pow_yaw 100"));
    TEST_ASSERT_TRUE(tuning_parse_line("pitch_deviation 5"));
    TEST_ASSERT_TRUE(tuning_parse_line("roll_deviation 2"));
    GAINS e = {
        .kp_h = 150.0f, .kd_h = 90.0f, .kp_vel = 0.08f, .kp_angle = 3.0f,
        .kp_rate = 7.0f, .ki_rate = 1.0f, .kp_yaw_rate = 4.0f,
        .max_rate = 150.0f, .max_pow_rp = 300.0f, .base = 2390.0f,
        .alpha = 0.9f, .max_angle = 0.3f, .max_pow_yaw = 100.0f, .pitch_deviation = 5.0f,
        .roll_deviation = 2.0f,
    };
    { GAINS a = now(); assert_gains_equal(&e, &a); }
}

/* The console also changes the flight and odometry parameters. */
void test_cmd_flight_and_odometry_parameters(void) {
    TEST_ASSERT_TRUE(tuning_parse_line("v_max 0.4"));
    TEST_ASSERT_TRUE(tuning_parse_line("k_drag 0.2"));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, get_max_velocity());
    odom_params_t o;
    get_odom_params(&o);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.2f, o.k_drag);
    { GAINS a = now(); assert_gains_equal(&START, &a); }
}

void test_cmd_kd_h_only_changes_kd_h(void) {
    TEST_ASSERT_TRUE(tuning_parse_line("kd_h 250"));
    GAINS e = START;
    e.kd_h = 250.0f;
    { GAINS a = now(); assert_gains_equal(&e, &a); }
}

void test_cmd_equal_sign_and_upper_case(void) {
    TEST_ASSERT_TRUE(tuning_parse_line("KP_ANGLE=1.5"));
    TEST_ASSERT_EQUAL_FLOAT(1.5f, now().kp_angle);
}

void test_cmd_extra_spaces(void) {
    TEST_ASSERT_TRUE(tuning_parse_line("   kp_h    120  "));
    TEST_ASSERT_EQUAL_FLOAT(120.0f, now().kp_h);
}

void test_cmd_get_and_help_do_not_change_gains(void) {
    TEST_ASSERT_TRUE(tuning_parse_line("get"));
    TEST_ASSERT_TRUE(tuning_parse_line("help"));
    { GAINS a = now(); assert_gains_equal(&START, &a); }
}

void test_cmd_reset_goes_back_to_the_menuconfig_values(void) {
    init_tuning();
    tuning_parse_line("kp_rate 1");
    tuning_parse_line("v_max 2");
    TEST_ASSERT_TRUE(tuning_parse_line("reset"));
    params_t d, p;
    get_default_params(&d);
    get_all_params(&p);
    TEST_ASSERT_EQUAL_MEMORY(&d, &p, sizeof d);
}

/* ------------------------------------------------------------------ */
/*                          INVALID COMMANDS                          */
/* ------------------------------------------------------------------ */
void test_bad_unknown_name(void) {
    TEST_ASSERT_FALSE(tuning_parse_line("kp_foo 3"));
    { GAINS a = now(); assert_gains_equal(&START, &a); }
}

void test_bad_missing_value(void) {
    TEST_ASSERT_FALSE(tuning_parse_line("kp_rate"));
    { GAINS a = now(); assert_gains_equal(&START, &a); }
}

void test_bad_not_a_number(void) {
    TEST_ASSERT_FALSE(tuning_parse_line("kp_rate abc"));
    TEST_ASSERT_FALSE(tuning_parse_line("kp_rate 3x"));
    TEST_ASSERT_FALSE(tuning_parse_line("kp_rate nan"));
    TEST_ASSERT_FALSE(tuning_parse_line("kp_rate inf"));
    { GAINS a = now(); assert_gains_equal(&START, &a); }
}

void test_bad_out_of_range(void) {
    TEST_ASSERT_FALSE(tuning_parse_line("kp_rate -1"));
    TEST_ASSERT_FALSE(tuning_parse_line("base 5000"));
    TEST_ASSERT_FALSE(tuning_parse_line("kd_h -1"));
    TEST_ASSERT_FALSE(tuning_parse_line("kd_h 5001"));
    TEST_ASSERT_FALSE(tuning_parse_line("alpha 1.5"));
    TEST_ASSERT_FALSE(tuning_parse_line("k_drag 0"));
    TEST_ASSERT_FALSE(tuning_parse_line("v_max 11"));
    { GAINS a = now(); assert_gains_equal(&START, &a); }
}

void test_bad_empty_line(void) {
    TEST_ASSERT_FALSE(tuning_parse_line(""));
    TEST_ASSERT_FALSE(tuning_parse_line("    "));
}

void test_bad_very_long_line_does_not_crash(void) {
    char line[300];
    memset(line, 'a', sizeof line - 1);
    line[sizeof line - 1] = '\0';
    TEST_ASSERT_FALSE(tuning_parse_line(line));
    { GAINS a = now(); assert_gains_equal(&START, &a); }
}

/* ------------------------------------------------------------------ */
/*                               TASK                                 */
/* ------------------------------------------------------------------ */
void test_task_created_with_the_menuconfig_values(void) {
    init_tuning();
    const mock_task_t *t = mock_find_task("tuning_task");
    TEST_ASSERT_NOT_NULL(t);
    TEST_ASSERT_EQUAL_UINT32(CONFIG_TUNING_TASK_STACK, t->stack);
    TEST_ASSERT_EQUAL_UINT(CONFIG_TUNING_TASK_PRIO, t->prio);
}

void test_task_init_twice_creates_one_task(void) {
    init_tuning();
    init_tuning();
    TEST_ASSERT_EQUAL_INT(1, mock_task_count);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_cmd_change_one_gain);
    RUN_TEST(test_cmd_every_gain_name);
    RUN_TEST(test_cmd_flight_and_odometry_parameters);
    RUN_TEST(test_cmd_kd_h_only_changes_kd_h);
    RUN_TEST(test_cmd_equal_sign_and_upper_case);
    RUN_TEST(test_cmd_extra_spaces);
    RUN_TEST(test_cmd_get_and_help_do_not_change_gains);
    RUN_TEST(test_cmd_reset_goes_back_to_the_menuconfig_values);
    RUN_TEST(test_bad_unknown_name);
    RUN_TEST(test_bad_missing_value);
    RUN_TEST(test_bad_not_a_number);
    RUN_TEST(test_bad_out_of_range);
    RUN_TEST(test_bad_empty_line);
    RUN_TEST(test_bad_very_long_line_does_not_crash);
    RUN_TEST(test_task_created_with_the_menuconfig_values);
    RUN_TEST(test_task_init_twice_creates_one_task);
    return UNITY_END();
}
