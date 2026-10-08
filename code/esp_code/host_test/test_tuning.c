/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for tuning.c (host, no hardware). get_gains() / set_gains()
 *   of the attitude controller are replaced by a fake GAINS here, so only
 *   the command parser and the task creation are tested.
 *
 *   The .c file is included directly. Another version can be tested with:
 *       make run T=tuning MODULE_SRC=path/to/tuning.c
 *
 * Functions:
 *   - setUp() / tearDown(): reset the mocks, the module and the gains.
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

/* Fake gains of the attitude controller */
static GAINS fake_gains;
static const GAINS START = { 100.0f, 2.0f, 10.0f, 200.0f, 400.0f, 2400.0f };

void get_gains(GAINS *out) { *out = fake_gains; }
void set_gains(const GAINS *in) { fake_gains = *in; }

static void assert_gains_equal(const GAINS *e, const GAINS *a) {
    TEST_ASSERT_EQUAL_FLOAT(e->kp_h, a->kp_h);
    TEST_ASSERT_EQUAL_FLOAT(e->kp_angle, a->kp_angle);
    TEST_ASSERT_EQUAL_FLOAT(e->kp_rate, a->kp_rate);
    TEST_ASSERT_EQUAL_FLOAT(e->max_rate, a->max_rate);
    TEST_ASSERT_EQUAL_FLOAT(e->max_pow_rpy, a->max_pow_rpy);
    TEST_ASSERT_EQUAL_FLOAT(e->base, a->base);
}

void setUp(void) {
    mock_reset_all();
    is_init = false;
    fake_gains = START;
}

void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                             COMMANDS                               */
/* ------------------------------------------------------------------ */
void test_cmd_change_one_gain(void) {
    TEST_ASSERT_TRUE(tuning_parse_line("kp_rate 8.5"));
    GAINS e = START;
    e.kp_rate = 8.5f;
    assert_gains_equal(&e, &fake_gains);
}

void test_cmd_every_gain_name(void) {
    TEST_ASSERT_TRUE(tuning_parse_line("kp_h 150"));
    TEST_ASSERT_TRUE(tuning_parse_line("kp_angle 3"));
    TEST_ASSERT_TRUE(tuning_parse_line("kp_rate 7"));
    TEST_ASSERT_TRUE(tuning_parse_line("max_rate 150"));
    TEST_ASSERT_TRUE(tuning_parse_line("max_pow 300"));
    TEST_ASSERT_TRUE(tuning_parse_line("base 2390"));
    GAINS e = { 150.0f, 3.0f, 7.0f, 150.0f, 300.0f, 2390.0f };
    assert_gains_equal(&e, &fake_gains);
}

void test_cmd_equal_sign_and_upper_case(void) {
    TEST_ASSERT_TRUE(tuning_parse_line("KP_ANGLE=1.5"));
    TEST_ASSERT_EQUAL_FLOAT(1.5f, fake_gains.kp_angle);
}

void test_cmd_extra_spaces(void) {
    TEST_ASSERT_TRUE(tuning_parse_line("   kp_h    120  "));
    TEST_ASSERT_EQUAL_FLOAT(120.0f, fake_gains.kp_h);
}

void test_cmd_get_and_help_do_not_change_gains(void) {
    TEST_ASSERT_TRUE(tuning_parse_line("get"));
    TEST_ASSERT_TRUE(tuning_parse_line("help"));
    assert_gains_equal(&START, &fake_gains);
}

void test_cmd_reset_goes_back_to_the_gains_at_init(void) {
    init_tuning();
    tuning_parse_line("kp_rate 1");
    tuning_parse_line("base 2000");
    TEST_ASSERT_TRUE(tuning_parse_line("reset"));
    assert_gains_equal(&START, &fake_gains);
}

/* ------------------------------------------------------------------ */
/*                          INVALID COMMANDS                          */
/* ------------------------------------------------------------------ */
void test_bad_unknown_name(void) {
    TEST_ASSERT_FALSE(tuning_parse_line("kp_foo 3"));
    assert_gains_equal(&START, &fake_gains);
}

void test_bad_missing_value(void) {
    TEST_ASSERT_FALSE(tuning_parse_line("kp_rate"));
    assert_gains_equal(&START, &fake_gains);
}

void test_bad_not_a_number(void) {
    TEST_ASSERT_FALSE(tuning_parse_line("kp_rate abc"));
    TEST_ASSERT_FALSE(tuning_parse_line("kp_rate 3x"));
    TEST_ASSERT_FALSE(tuning_parse_line("kp_rate nan"));
    TEST_ASSERT_FALSE(tuning_parse_line("kp_rate inf"));
    assert_gains_equal(&START, &fake_gains);
}

void test_bad_out_of_range(void) {
    TEST_ASSERT_FALSE(tuning_parse_line("kp_rate -1"));
    TEST_ASSERT_FALSE(tuning_parse_line("base 5000"));
    assert_gains_equal(&START, &fake_gains);
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
    assert_gains_equal(&START, &fake_gains);
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
    RUN_TEST(test_cmd_equal_sign_and_upper_case);
    RUN_TEST(test_cmd_extra_spaces);
    RUN_TEST(test_cmd_get_and_help_do_not_change_gains);
    RUN_TEST(test_cmd_reset_goes_back_to_the_gains_at_init);
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
