/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for odometry_estimator.c (host, no hardware). The estimator
 *   is not implemented yet (estimate_odom() is empty), so these tests check
 *   the init, the task and that it does not break the global state. The
 *   tests of the estimation are IGNORED until it is implemented: remove the
 *   TEST_IGNORE_MESSAGE() line when you write estimate_odom().
 *
 *   The .c file is included directly. Another version can be tested with:
 *       make run T=odometry MODULE_SRC=path/to/odometry_estimator.c
 *
 * Functions:
 *   - setUp() / tearDown(): reset the mocks, the module and the state.
 *   - test_init_*: odom_estimator_init() and odom_estimator_test().
 *   - test_task_*: the FreeRTOS task.
 *   - test_odom_*: the estimation (ignored for now).
 *   - main(): runs all the tests.
 */
#include "unity.h"
#include "test_helpers.h"
#include "mocks/mock_reset.h"

#ifndef MODULE_SRC
#define MODULE_SRC "../components/flight_control/odometry_estimator.c"
#endif
#include MODULE_SRC
#include "state.h"

void setUp(void) {
    mock_reset_all();
    is_init = false;
    state_init();
}

void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                               INIT                                 */
/* ------------------------------------------------------------------ */
void test_init_test_is_false_before_init(void) {
    TEST_ASSERT_FALSE(odom_estimator_test());
}

void test_init_test_is_true_after_init(void) {
    odom_estimator_init();
    TEST_ASSERT_TRUE(odom_estimator_test());
}

void test_init_creates_the_task_with_the_menuconfig_values(void) {
    odom_estimator_init();
    const mock_task_t *t = mock_find_task("odom_task");
    TEST_ASSERT_NOT_NULL(t);
    TEST_ASSERT_EQUAL_UINT32(CONFIG_ODOM_TASK_STACK, t->stack);
    TEST_ASSERT_EQUAL_UINT(CONFIG_ODOM_TASK_PRIO, t->prio);
}

void test_init_twice_creates_one_task(void) {
    odom_estimator_init();
    odom_estimator_init();
    TEST_ASSERT_EQUAL_INT(1, mock_task_count);
}

/* ------------------------------------------------------------------ */
/*                                TASK                                */
/* ------------------------------------------------------------------ */
void test_task_period_is_10_ms_100_hz(void) {
    TEST_ASSERT_EQUAL_INT(10, ODOM_TASK_PERIOD_MS);
    int delays = mock_run_task(odom_task, NULL, 25);
    TEST_ASSERT_EQUAL_INT(25, delays);
    TEST_ASSERT_EQUAL_UINT64(25 * 10, mock_delay_total_ms);
}

void test_task_does_not_change_the_state_yet(void) {
    set_position(1.0f, 2.0f);
    set_vel_lin(0.1f, 0.2f, 0.3f);
    mock_run_task(odom_task, NULL, 10);
    vec3_t p, v;
    get_position(&p);
    get_vel_lin(&v);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, p.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, p.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, v.z);
}

/* ------------------------------------------------------------------ */
/*                       ESTIMATION (TODO)                            */
/* ------------------------------------------------------------------ */
void test_odom_still_drone_does_not_move(void) {
    TEST_IGNORE_MESSAGE("estimate_odom() is not implemented yet");
    set_acc_lin(0, 0, 1.0f);   /* at rest: only gravity */
    for (int i = 0; i < 100; i++) estimate_odom();
    vec3_t p;
    get_position(&p);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, p.x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, p.y);
}

void test_odom_constant_velocity_moves_the_position(void) {
    TEST_IGNORE_MESSAGE("estimate_odom() is not implemented yet");
    set_vel_lin(1.0f, 0, 0);
    for (int i = 0; i < 100; i++) estimate_odom();   /* 100 x 10 ms = 1 s */
    vec3_t p;
    get_position(&p);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, p.x);
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_init_test_is_false_before_init);
    RUN_TEST(test_init_test_is_true_after_init);
    RUN_TEST(test_init_creates_the_task_with_the_menuconfig_values);
    RUN_TEST(test_init_twice_creates_one_task);

    RUN_TEST(test_task_period_is_10_ms_100_hz);
    RUN_TEST(test_task_does_not_change_the_state_yet);

    RUN_TEST(test_odom_still_drone_does_not_move);
    RUN_TEST(test_odom_constant_velocity_moves_the_position);

    return UNITY_END();
}
