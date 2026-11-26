/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for odometry_estimator.c (host, no hardware). They check the
 *   init, the task and estimate_odom(): orientation with the Madgwick filter
 *   (quaternions.c), horizontal velocity from the rotor drag in world axes
 *   (v_body = -a_body / K_DRAG, only in flight, low pass TAU_VEL), vertical
 *   velocity from the attitude controller (get_vel_z()) and the position.
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
#define DT_NS 10000000LL /* 10 ms, period of the task */

/* Default rotor drag (parameters.c, menuconfig x1000) */
#define K_DRAG (CONFIG_K_DRAG / 1000.0f)

/* Velocity (m/s) that the drag model gives for a body acceleration in g. */
#define DRAG_VEL(acc_g) (-(acc_g) * K_G / K_DRAG)

/* Part of the error of the velocity corrected by the low pass in one step. */
#define BETA_STEP ((DT_NS / 1e9f) / (TAU_VEL + DT_NS / 1e9f))

/* Enough steps for the low pass to converge (10 TAU_VEL). */
#define SETTLE_STEPS 100

/* The Madgwick filter takes a bit of a constant acceleration as tilt, so
 * the tests allow an error of 3 %. */
#define VEL_TOL(v) (0.03f * fabsf(v))

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
    t_now = 1000000000LL; /* 1 s */
    last_t_imu = t_now;
    set_time_imu(t_now);
    set_acc_lin(0, 0, 1.0f); /* level and still: only gravity */
    set_vel_ang(0, 0, 0);
}

void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                               INIT                                 */
/* ------------------------------------------------------------------ */
void test_init_test_is_false_before_init(void) {
    TEST_ASSERT_FALSE(odom_estimator_test());
}

/* The mock does not run odom_task: this hook does one iteration of the task
 * every 10 ms that odom_estimator_test() waits. */
static void odom_task_runs(TickType_t ticks) {
    odom_iter += ticks / ODOM_TASK_PERIOD_MS;
}

void test_init_test_is_true_after_init(void) {
    init_odom_estimator();
    mock_delay_hook = odom_task_runs;
    TEST_ASSERT_TRUE(odom_estimator_test());
}

/* odom_task dead or stuck: no iterations while the test waits */
void test_init_test_is_false_if_the_task_does_not_run(void) {
    init_odom_estimator();
    TEST_ASSERT_FALSE(odom_estimator_test());
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

/* Drag: flying forward the acceleration is backwards. */
void test_odom_forward_drag_gives_forward_velocity(void) {
    set_sm_state(HOVERING);
    set_acc_lin(-0.01f, 0, 1.0f);
    for (int i = 0; i < SETTLE_STEPS; i++) odom_step();
    TEST_ASSERT_TRUE(DRAG_VEL(-0.01f) > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(VEL_TOL(DRAG_VEL(-0.01f)), DRAG_VEL(-0.01f), vel_x);
    TEST_ASSERT_FLOAT_WITHIN(VEL_TOL(DRAG_VEL(-0.01f)), 0.0f, vel_y);
}

void test_odom_lateral_drag_gives_lateral_velocity(void) {
    set_sm_state(HOVERING);
    set_acc_lin(0, 0.01f, 1.0f);
    for (int i = 0; i < SETTLE_STEPS; i++) odom_step();
    TEST_ASSERT_FLOAT_WITHIN(VEL_TOL(DRAG_VEL(0.01f)), DRAG_VEL(0.01f), vel_y);
    TEST_ASSERT_FLOAT_WITHIN(VEL_TOL(DRAG_VEL(0.01f)), 0.0f, vel_x);
}

/* It is not integrated: the velocity does not jump, it follows the low pass. */
void test_odom_velocity_follows_the_low_pass(void) {
    set_sm_state(HOVERING);
    set_acc_lin(-0.01f, 0, 1.0f);
    odom_step();
    float expected = BETA_STEP * DRAG_VEL(-0.01f);
    TEST_ASSERT_FLOAT_WITHIN(VEL_TOL(expected), expected, vel_x);
}

/* A constant acceleration gives a constant velocity, it does not drift. */
void test_odom_velocity_does_not_drift(void) {
    set_sm_state(HOVERING);
    set_acc_lin(-0.002f, 0, 1.0f);
    for (int i = 0; i < 2000; i++) odom_step(); /* 20 s */
    TEST_ASSERT_TRUE(fabsf(vel_x) <= fabsf(DRAG_VEL(0.002f)) * 1.05f);
}

/* The drag is only used in these states. */
void test_odom_drag_is_used_in_the_flight_states(void) {
    const sm_states_t flying[] = { TAKING_OFF, HOVERING, EXTERNAL_CONTROL, LANDING,
                                  EMERGENCY_LANDING };
    for (unsigned i = 0; i < sizeof flying / sizeof flying[0]; i++) {
        set_sm_state(flying[i]);
        vel_x = 0.0f;
        set_acc_lin(-0.01f, 0, 1.0f);
        odom_step();
        TEST_ASSERT_TRUE(vel_x > 0.0f);
    }
}

/* On the ground the accelerometer sees the gravity of the tilt, not drag. */
void test_odom_on_the_ground_the_acceleration_is_ignored(void) {
    set_sm_state(ARMING);
    set_acc_lin(-0.1f, 0.1f, 1.0f);
    for (int i = 0; i < SETTLE_STEPS; i++) odom_step();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vel_x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vel_y);
}

/* Out of flight the velocity goes back to 0 with the low pass. */
void test_odom_on_the_ground_the_velocity_goes_to_0(void) {
    set_sm_state(ARMING);
    vel_x = 1.0f;
    vel_y = -1.0f;
    for (int i = 0; i < 10; i++) odom_step();
    float expected = powf(1.0f - BETA_STEP, 10);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, expected, vel_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -expected, vel_y);
}

/* The velocity is rotated to world axes: with the drone turned 90 deg in
 * yaw, "forward" of the drone is +Y of the world. */
void test_odom_velocity_is_rotated_with_the_yaw(void) {
    set_sm_state(HOVERING);
    q = quat_yaw(90.0f);
    set_acc_lin(-0.01f, 0, 1.0f);
    odom_step();
    float expected = BETA_STEP * DRAG_VEL(-0.01f);
    TEST_ASSERT_FLOAT_WITHIN(VEL_TOL(expected), expected, vel_y);
    TEST_ASSERT_FLOAT_WITHIN(VEL_TOL(expected), 0.0f, vel_x);
}

void test_odom_first_sample_long_dt_is_ignored(void) {
    set_sm_state(HOVERING);
    last_t_imu = 0; /* after boot */
    set_acc_lin(0.5f, 0.5f, 1.0f);
    odom_step();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vel_x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vel_y);
}

void test_odom_zero_dt_does_not_change_the_velocity(void) {
    set_sm_state(HOVERING);
    set_acc_lin(0.5f, 0.5f, 1.0f);
    estimate_odom(); /* same IMU time */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vel_x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vel_y);
}

/* The position integrates the velocity. */
void test_odom_constant_velocity_moves_the_position(void) {
    set_sm_state(HOVERING);
    float v = DRAG_VEL(-0.01f);
    vel_x = v;
    set_acc_lin(-0.01f, 0, 1.0f);
    for (int i = 0; i < 100; i++) odom_step(); /* 100 x 10 ms = 1 s */
    vec3_t p;
    get_position(&p);
    TEST_ASSERT_FLOAT_WITHIN(VEL_TOL(v), v, p.x);
}

/* quaternion_update() needs the gyroscope in rad/s and the state has it in
 * deg/s: 1 s at 90 deg/s in yaw must give 90 deg. */
void test_odom_gyro_is_converted_to_rad_s(void) {
    set_vel_ang(0, 0, 90.0f); /* deg/s */
    for (int i = 0; i < 100; i++) odom_step();
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 90.0f, quaternion_get_yaw(&q));
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_init_test_is_false_before_init);
    RUN_TEST(test_init_test_is_true_after_init);
    RUN_TEST(test_init_test_is_false_if_the_task_does_not_run);
    RUN_TEST(test_init_creates_the_task_with_the_menuconfig_values);
    RUN_TEST(test_init_twice_creates_one_task);
    RUN_TEST(test_init_resets_the_quaternion_and_the_velocities);

    RUN_TEST(test_task_period_is_10_ms_100_hz);
    RUN_TEST(test_task_estimates_every_cycle);

    RUN_TEST(test_odom_saves_the_quaternion_in_the_state);
    RUN_TEST(test_odom_level_and_still_keeps_the_identity);
    RUN_TEST(test_odom_still_drone_does_not_move);
    RUN_TEST(test_odom_vel_z_comes_from_the_attitude_controller);
    RUN_TEST(test_odom_forward_drag_gives_forward_velocity);
    RUN_TEST(test_odom_lateral_drag_gives_lateral_velocity);
    RUN_TEST(test_odom_velocity_follows_the_low_pass);
    RUN_TEST(test_odom_velocity_does_not_drift);
    RUN_TEST(test_odom_drag_is_used_in_the_flight_states);
    RUN_TEST(test_odom_on_the_ground_the_acceleration_is_ignored);
    RUN_TEST(test_odom_on_the_ground_the_velocity_goes_to_0);
    RUN_TEST(test_odom_velocity_is_rotated_with_the_yaw);
    RUN_TEST(test_odom_first_sample_long_dt_is_ignored);
    RUN_TEST(test_odom_zero_dt_does_not_change_the_velocity);
    RUN_TEST(test_odom_constant_velocity_moves_the_position);
    RUN_TEST(test_odom_gyro_is_converted_to_rad_s);

    return UNITY_END();
}
