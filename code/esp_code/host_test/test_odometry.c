/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for odometry_estimator.c (host, no hardware). They check the
 *   init, the task and estimate_odom(): orientation with the Madgwick filter
 *   (quaternions.c), horizontal velocity integrating the acceleration in
 *   world axes (leaky integration, V_LEAK_TAU), vertical velocity from the
 *   attitude controller (get_vel_z()) and the position.
 *
 *   The real state.c and quaternions.c are linked. get_vel_z()
 *   (attitude_controller.c) is mocked.
 *
 *   Units (same as the drivers): acceleration in g (Z = +1 g at rest),
 *   angular velocity in deg/s, time in ns, velocity in m/s, position in m.
 *
 *   The .c file is included directly. Another version can be tested with:
 *       make run T=odometry MODULE_SRC=path/to/odometry_estimator.c
 *
 *   Tests that fail with the current code have [BUG] in their comment.
 *
 * Functions:
 *   - get_vel_z(): mock of attitude_controller.c.
 *   - odom_step(): one call to estimate_odom() 10 ms later.
 *   - setUp() / tearDown(): reset the mocks, the module and the state.
 *   - test_init_*: init_odom_estimator() and odom_estimator_test().
 *   - test_task_*: the FreeRTOS task.
 *   - test_odom_*: the estimation.
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

/* Gravity, without the one letter macro of the module */
static const float K_G = g;
#undef g

/* ------------------------------------------------------------------ */
/*                               MOCKS                                */
/* ------------------------------------------------------------------ */
static float vel_z_ret;

float get_vel_z() { return vel_z_ret; }

/* ------------------------------------------------------------------ */
/*                              HELPERS                               */
/* ------------------------------------------------------------------ */
#define DT_NS 10000000LL   /* 10 ms, period of the task */

/* Velocity after one step of acc_g (in g). The Madgwick filter takes a bit
 * of a constant acceleration as tilt (it cannot tell them apart), so the
 * tests allow a 3 % error in one step. */
#define ONE_STEP_VEL(acc_g) ((acc_g) * K_G * DT_NS / 1e9f)
#define ONE_STEP_TOL(acc_g) (0.03f * fabsf(ONE_STEP_VEL(acc_g)))

static int64_t t_now;

/* One estimation 10 ms later. */
static void odom_step(void) {
    t_now += DT_NS;
    set_time_imu(t_now);
    estimate_odom();
}

/* Quaternion of a rotation of yaw_deg around Z. */
static quat_t quat_yaw(float yaw_deg) {
    quat_t r = { 0.0f, 0.0f, sinf(RAD(yaw_deg) / 2), cosf(RAD(yaw_deg) / 2) };
    return r;
}

void setUp(void) {
    mock_reset_all();
    is_init = false;
    state_init();
    quaternion_init(&q);
    vel_x = vel_y = vel_z = 0.0f;
    x = y = 0.0f;
    vel_z_ret = 0.0f;
    t_now = 1000000000LL;   /* 1 s */
    last_t_imu = t_now;
    set_time_imu(t_now);
    set_acc_lin(0, 0, 1.0f);   /* level and still: only gravity */
    set_vel_ang(0, 0, 0);
}

void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                               INIT                                 */
/* ------------------------------------------------------------------ */
void test_init_test_is_false_before_init(void) {
    TEST_ASSERT_FALSE(odom_estimator_test());
}

void test_init_test_is_true_after_init(void) {
    init_odom_estimator();
    TEST_ASSERT_TRUE(odom_estimator_test());
}

void test_init_creates_the_task_with_the_menuconfig_values(void) {
    init_odom_estimator();
    const mock_task_t *t = mock_find_task("odom_task");
    TEST_ASSERT_NOT_NULL(t);
    TEST_ASSERT_EQUAL_UINT32(CONFIG_ODOM_TASK_STACK, t->stack);
    TEST_ASSERT_EQUAL_UINT(CONFIG_ODOM_TASK_PRIO, t->prio);
}

void test_init_twice_creates_one_task(void) {
    init_odom_estimator();
    init_odom_estimator();
    TEST_ASSERT_EQUAL_INT(1, mock_task_count);
}

void test_init_resets_the_quaternion_and_the_velocities(void) {
    q = quat_yaw(45.0f);
    vel_x = 1.0f;
    vel_y = 2.0f;
    vel_z = 3.0f;
    last_t_imu = 5.0f;
    init_odom_estimator();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, q.w);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, q.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, q.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, q.z);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vel_x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vel_y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vel_z);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, last_t_imu);
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

void test_task_estimates_every_cycle(void) {
    vel_z_ret = 0.4f;
    mock_run_task(odom_task, NULL, 3);
    vec3_t v;
    get_vel_lin(&v);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, v.z);
}

/* ------------------------------------------------------------------ */
/*                             ESTIMATION                             */
/* ------------------------------------------------------------------ */
void test_odom_saves_the_quaternion_in_the_state(void) {
    odom_step();
    quat_t s;
    get_orientation(&s);
    TEST_ASSERT_EQUAL_FLOAT(q.w, s.w);
    TEST_ASSERT_EQUAL_FLOAT(q.x, s.x);
    TEST_ASSERT_EQUAL_FLOAT(q.y, s.y);
    TEST_ASSERT_EQUAL_FLOAT(q.z, s.z);
}

void test_odom_level_and_still_keeps_the_identity(void) {
    for (int i = 0; i < 100; i++) odom_step();
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, q.w);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, q.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, q.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, q.z);
}

void test_odom_still_drone_does_not_move(void) {
    for (int i = 0; i < 100; i++) odom_step();
    vec3_t p, v;
    get_position(&p);
    get_vel_lin(&v);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, v.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, v.y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, p.x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, p.y);
}

void test_odom_vel_z_comes_from_the_attitude_controller(void) {
    vel_z_ret = -0.25f;
    odom_step();
    vec3_t v;
    get_vel_lin(&v);
    TEST_ASSERT_EQUAL_FLOAT(-0.25f, v.z);
    TEST_ASSERT_EQUAL_FLOAT(-0.25f, vel_z);
}

void test_odom_forward_acceleration_gives_forward_velocity(void) {
    set_acc_lin(0.1f, 0, 1.0f);   /* +0.1 g forward */
    odom_step();
    vec3_t v;
    get_vel_lin(&v);
    TEST_ASSERT_FLOAT_WITHIN(ONE_STEP_TOL(0.1f), ONE_STEP_VEL(0.1f), v.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, v.y);
}

void test_odom_lateral_acceleration_gives_lateral_velocity(void) {
    set_acc_lin(0, -0.1f, 1.0f);
    odom_step();
    vec3_t v;
    get_vel_lin(&v);
    TEST_ASSERT_FLOAT_WITHIN(ONE_STEP_TOL(-0.1f), ONE_STEP_VEL(-0.1f), v.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, v.x);
}

void test_odom_velocity_is_integrated(void) {
    /* 0.1 g during 100 ms -> 0.0981 m/s. The Madgwick filter slowly takes
     * the constant acceleration as tilt, so it gives a bit less (~9 %). */
    set_acc_lin(0.1f, 0, 1.0f);
    for (int i = 0; i < 10; i++) odom_step();
    TEST_ASSERT_FLOAT_WITHIN(0.15f * 0.0981f, 0.1f * K_G * 0.1f, vel_x);
}

/* The acceleration is rotated to world axes: with the drone turned 90 deg
 * in yaw, "forward" of the drone is +Y of the world. */
void test_odom_acceleration_is_rotated_with_the_yaw(void) {
    q = quat_yaw(90.0f);
    set_acc_lin(0.1f, 0, 1.0f);
    odom_step();
    TEST_ASSERT_FLOAT_WITHIN(ONE_STEP_TOL(0.1f), ONE_STEP_VEL(0.1f), vel_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, vel_x);
}

/* Tilted and still: the gravity seen by the accelerometer is not a
 * horizontal acceleration once it is rotated to world axes. Without the
 * rotation it would be sin(10) g = 1.7 m/s after 1 s; the Madgwick step
 * (BETA * dt) makes q oscillate a bit, so a few mm/s are allowed. */
void test_odom_tilted_and_still_does_not_move(void) {
    float r = RAD(10.0f);
    q.w = cosf(r / 2); q.x = sinf(r / 2); q.y = 0; q.z = 0;   /* roll 10 deg */
    set_acc_lin(0, sinf(r), cosf(r));
    for (int i = 0; i < 100; i++) odom_step();
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, vel_x);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, vel_y);
}

void test_odom_first_sample_long_dt_is_ignored(void) {
    last_t_imu = 0;   /* after boot */
    set_acc_lin(0.5f, 0.5f, 1.0f);
    odom_step();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vel_x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vel_y);
}

void test_odom_zero_dt_does_not_integrate(void) {
    set_acc_lin(0.5f, 0.5f, 1.0f);
    estimate_odom();   /* same IMU time */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vel_x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vel_y);
}

/* The position accumulates the velocity. Still, the velocity forgets
 * with V_LEAK_TAU: 1 m/s during 1 s moves tau * (1 - e^(-1 / tau)) m. */
void test_odom_constant_velocity_moves_the_position(void) {
    vel_x = 1.0f;
    for (int i = 0; i < 100; i++) odom_step();   /* 100 x 10 ms = 1 s */
    vec3_t p;
    get_position(&p);
    float expected = V_LEAK_TAU * (1.0f - expf(-1.0f / V_LEAK_TAU));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, expected, p.x);
}

/* Without acceleration the velocity goes back to 0: after V_LEAK_TAU it
 * keeps 1 / e (37 %). */
void test_odom_velocity_forgets_with_v_leak_tau(void) {
    vel_x = 1.0f;
    vel_y = -1.0f;
    int n = (int)(V_LEAK_TAU / (DT_NS / 1e9f));
    for (int i = 0; i < n; i++) odom_step();
    TEST_ASSERT_FLOAT_WITHIN(0.02f, expf(-1.0f), vel_x);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, -expf(-1.0f), vel_y);
}

/* A bias of the accelerometer does not grow without limit: the velocity
 * stays under bias * V_LEAK_TAU (the Madgwick filter also takes part of
 * the bias as tilt, so it can be less). */
void test_odom_accel_bias_gives_a_limited_velocity(void) {
    set_acc_lin(0.002f, 0, 1.0f);   /* 0.002 g = 0.02 m/s^2 of bias */
    for (int i = 0; i < 2000; i++) odom_step();   /* 20 s = 8 tau */
    TEST_ASSERT_TRUE(fabsf(vel_x) <= 0.002f * K_G * V_LEAK_TAU * 1.05f);
}

/* [BUG] quaternion_update() needs the gyroscope in rad/s, but the state
 * has it in deg/s and it is passed without conversion: the orientation
 * turns 57.3 times faster. 1 s at 90 deg/s in yaw must give 90 deg. */
void test_odom_gyro_is_converted_to_rad_s(void) {
    set_vel_ang(0, 0, 90.0f);   /* deg/s */
    for (int i = 0; i < 100; i++) odom_step();
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 90.0f, quaternion_get_yaw(&q));
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_init_test_is_false_before_init);
    RUN_TEST(test_init_test_is_true_after_init);
    RUN_TEST(test_init_creates_the_task_with_the_menuconfig_values);
    RUN_TEST(test_init_twice_creates_one_task);
    RUN_TEST(test_init_resets_the_quaternion_and_the_velocities);

    RUN_TEST(test_task_period_is_10_ms_100_hz);
    RUN_TEST(test_task_estimates_every_cycle);

    RUN_TEST(test_odom_saves_the_quaternion_in_the_state);
    RUN_TEST(test_odom_level_and_still_keeps_the_identity);
    RUN_TEST(test_odom_still_drone_does_not_move);
    RUN_TEST(test_odom_vel_z_comes_from_the_attitude_controller);
    RUN_TEST(test_odom_forward_acceleration_gives_forward_velocity);
    RUN_TEST(test_odom_lateral_acceleration_gives_lateral_velocity);
    RUN_TEST(test_odom_velocity_is_integrated);
    RUN_TEST(test_odom_acceleration_is_rotated_with_the_yaw);
    RUN_TEST(test_odom_tilted_and_still_does_not_move);
    RUN_TEST(test_odom_first_sample_long_dt_is_ignored);
    RUN_TEST(test_odom_zero_dt_does_not_integrate);
    RUN_TEST(test_odom_constant_velocity_moves_the_position);
    RUN_TEST(test_odom_velocity_forgets_with_v_leak_tau);
    RUN_TEST(test_odom_accel_bias_gives_a_limited_velocity);
    RUN_TEST(test_odom_gyro_is_converted_to_rad_s);

    return UNITY_END();
}
