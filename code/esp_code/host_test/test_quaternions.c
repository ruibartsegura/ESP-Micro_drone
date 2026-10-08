/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for quaternions.c, the Madgwick filter (host, no hardware).
 *   They check the helpers, the Euler angles, the integration of the
 *   gyroscope, the correction with the accelerometer and that the
 *   quaternion always stays normalised.
 *
 *   Conventions (same as the code): gyroscope in rad/s, dt in s, angles
 *   returned in degrees, ZYX order (yaw, pitch, roll). With a level drone
 *   the accelerometer reads (0, 0, +1 g); for a roll r and a pitch p it
 *   reads (-sin p, cos p sin r, cos p cos r).
 *
 *   The .c file is included directly, so the static helpers can be
 *   tested. Another version can be tested with:
 *       make run T=quaternions MODULE_SRC=path/to/quaternions.c
 *
 * Functions:
 *   - quat_from_euler(): builds a quaternion from roll, pitch, yaw (degrees).
 *   - run_filter(): calls quaternion_update() N times with the same input.
 *   - setUp() / tearDown(): reset the mocks before each test.
 *   - test_helpers_*: inv_sqrt() and wrap_angle().
 *   - test_euler_*: quaternion_get_roll/pitch/yaw().
 *   - test_gyro_*: integration of the gyroscope.
 *   - test_accel_*: correction with the accelerometer.
 *   - test_norm_*: the quaternion stays normalised.
 *   - main(): runs all the tests.
 */
#include "unity.h"
#include "test_helpers.h"
#include "mocks/mock_reset.h"

#ifndef MODULE_SRC
#define MODULE_SRC "../components/drivers/imu/quaternions.c"
#endif
#include MODULE_SRC

static Quaternion quat_from_euler(float roll_deg, float pitch_deg, float yaw_deg) {
    float cr = cosf(RAD(roll_deg) / 2), sr = sinf(RAD(roll_deg) / 2);
    float cp = cosf(RAD(pitch_deg) / 2), sp = sinf(RAD(pitch_deg) / 2);
    float cy = cosf(RAD(yaw_deg) / 2), sy = sinf(RAD(yaw_deg) / 2);
    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

static float norm(const Quaternion *q) {
    return sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
}

static void run_filter(Quaternion *q, int steps, float gx, float gy, float gz,
                       float ax, float ay, float az, float dt) {
    for (int i = 0; i < steps; i++) quaternion_update(q, gx, gy, gz, ax, ay, az, dt);
}

/* Accelerometer reading (in g) for a drone with this roll and pitch. */
static void gravity(float roll_deg, float pitch_deg, float *ax, float *ay, float *az) {
    *ax = -sinf(RAD(pitch_deg));
    *ay = cosf(RAD(pitch_deg)) * sinf(RAD(roll_deg));
    *az = cosf(RAD(pitch_deg)) * cosf(RAD(roll_deg));
}

void setUp(void) { mock_reset_all(); }
void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                              HELPERS                               */
/* ------------------------------------------------------------------ */
void test_helpers_inv_sqrt_values(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, inv_sqrt(4.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, inv_sqrt(1.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f, inv_sqrt(0.01f));
}

void test_helpers_inv_sqrt_zero_and_negative_are_zero(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, inv_sqrt(0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, inv_sqrt(-1.0f));
}

void test_helpers_wrap_angle_inside_range_is_not_changed(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, wrap_angle(0.0f));
    TEST_ASSERT_EQUAL_FLOAT(179.0f, wrap_angle(179.0f));
    TEST_ASSERT_EQUAL_FLOAT(-179.0f, wrap_angle(-179.0f));
    TEST_ASSERT_EQUAL_FLOAT(180.0f, wrap_angle(180.0f));
    TEST_ASSERT_EQUAL_FLOAT(-180.0f, wrap_angle(-180.0f));
}

void test_helpers_wrap_angle_outside_range(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -170.0f, wrap_angle(190.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 170.0f, wrap_angle(-190.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, wrap_angle(360.0f));
}

/* ------------------------------------------------------------------ */
/*                           EULER ANGLES                             */
/* ------------------------------------------------------------------ */
void test_euler_init_is_identity(void) {
    Quaternion q = { 9, 9, 9, 9 };
    quaternion_init(&q);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, q.w);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, q.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, q.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, q.z);
}

void test_euler_identity_is_zero_angles(void) {
    Quaternion q;
    quaternion_init(&q);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, quaternion_get_roll(&q));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, quaternion_get_pitch(&q));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, quaternion_get_yaw(&q));
}

void test_euler_round_trip_on_a_grid(void) {
    for (int r = -150; r <= 150; r += 30)
        for (int p = -80; p <= 80; p += 20)
            for (int y = -150; y <= 150; y += 50) {
                Quaternion q = quat_from_euler(r, p, y);
                TEST_ASSERT_FLOAT_WITHIN(0.01f, r, quaternion_get_roll(&q));
                TEST_ASSERT_FLOAT_WITHIN(0.01f, p, quaternion_get_pitch(&q));
                TEST_ASSERT_FLOAT_WITHIN(0.01f, y, quaternion_get_yaw(&q));
            }
}

void test_euler_angles_are_in_degrees(void) {
    Quaternion q = quat_from_euler(45.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 45.0f, quaternion_get_roll(&q));
}

void test_euler_pitch_90_does_not_give_nan(void) {
    Quaternion q = quat_from_euler(0.0f, 90.0f, 0.0f);
    q.y *= 1.0001f;   /* rounding error: sinp a bit over 1 */
    float p = quaternion_get_pitch(&q);
    TEST_ASSERT_FALSE(isnan(p));
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, p);
}

void test_euler_pitch_minus_90_does_not_give_nan(void) {
    Quaternion q = quat_from_euler(0.0f, -90.0f, 0.0f);
    q.y *= 1.0001f;
    float p = quaternion_get_pitch(&q);
    TEST_ASSERT_FALSE(isnan(p));
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -90.0f, p);
}

void test_euler_angles_are_always_in_range(void) {
    for (int i = 0; i < 1000; i++) {
        Quaternion q = { cosf(i * 0.37f), sinf(i * 0.11f), cosf(i * 0.23f), sinf(i * 0.71f) };
        float n = norm(&q);
        q.w /= n; q.x /= n; q.y /= n; q.z /= n;
        float r = quaternion_get_roll(&q), p = quaternion_get_pitch(&q), y = quaternion_get_yaw(&q);
        TEST_ASSERT_TRUE(r >= -180.0f && r <= 180.0f);
        TEST_ASSERT_TRUE(p >= -90.0f && p <= 90.0f);
        TEST_ASSERT_TRUE(y >= -180.0f && y <= 180.0f);
    }
}

/* ------------------------------------------------------------------ */
/*                         GYRO INTEGRATION                           */
/* ------------------------------------------------------------------ */
/* With the accelerometer at 0 the correction is skipped: only the gyro. */
void test_gyro_no_rotation_keeps_identity(void) {
    Quaternion q;
    quaternion_init(&q);
    run_filter(&q, 100, 0, 0, 0, 0, 0, 0, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, q.w);
}

void test_gyro_yaw_90_dps_for_1_s_is_90_deg(void) {
    Quaternion q;
    quaternion_init(&q);
    run_filter(&q, 1000, 0, 0, RAD(90.0f), 0, 0, 0, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, quaternion_get_yaw(&q));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, quaternion_get_roll(&q));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, quaternion_get_pitch(&q));
}

void test_gyro_roll_30_dps_for_1_s_is_30_deg(void) {
    Quaternion q;
    quaternion_init(&q);
    run_filter(&q, 1000, RAD(30.0f), 0, 0, 0, 0, 0, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 30.0f, quaternion_get_roll(&q));
}

void test_gyro_pitch_minus_20_dps_for_1_s_is_minus_20_deg(void) {
    Quaternion q;
    quaternion_init(&q);
    run_filter(&q, 1000, 0, RAD(-20.0f), 0, 0, 0, 0, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, -20.0f, quaternion_get_pitch(&q));
}

void test_gyro_result_does_not_depend_on_the_step(void) {
    Quaternion a, b;
    quaternion_init(&a);
    quaternion_init(&b);
    run_filter(&a, 100,  0, 0, RAD(45.0f), 0, 0, 0, 0.01f);
    run_filter(&b, 1000, 0, 0, RAD(45.0f), 0, 0, 0, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, quaternion_get_yaw(&b), quaternion_get_yaw(&a));
}

void test_gyro_zero_dt_does_not_change_the_quaternion(void) {
    Quaternion q = quat_from_euler(10.0f, 20.0f, 30.0f);
    Quaternion before = q;
    quaternion_update(&q, 1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.9f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, before.w, q.w);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, before.x, q.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, before.y, q.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, before.z, q.z);
}

/* ------------------------------------------------------------------ */
/*                        ACCEL CORRECTION                            */
/* ------------------------------------------------------------------ */
void test_accel_level_reading_keeps_identity(void) {
    Quaternion q;
    quaternion_init(&q);
    run_filter(&q, 1000, 0, 0, 0, 0, 0, 1.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, quaternion_get_roll(&q));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, quaternion_get_pitch(&q));
}

void test_accel_converges_to_roll_30(void) {
    float ax, ay, az;
    Quaternion q;
    quaternion_init(&q);
    gravity(30.0f, 0.0f, &ax, &ay, &az);
    run_filter(&q, 3000, 0, 0, 0, ax, ay, az, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 30.0f, quaternion_get_roll(&q));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, quaternion_get_pitch(&q));
}

void test_accel_converges_to_pitch_minus_20(void) {
    float ax, ay, az;
    Quaternion q;
    quaternion_init(&q);
    gravity(0.0f, -20.0f, &ax, &ay, &az);
    run_filter(&q, 3000, 0, 0, 0, ax, ay, az, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -20.0f, quaternion_get_pitch(&q));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, quaternion_get_roll(&q));
}

void test_accel_units_do_not_matter(void) {
    /* The accelerometer is normalised: g and m/s^2 give the same result */
    float ax, ay, az;
    Quaternion a, b;
    quaternion_init(&a);
    quaternion_init(&b);
    gravity(15.0f, 10.0f, &ax, &ay, &az);
    run_filter(&a, 500, 0, 0, 0, ax, ay, az, 0.01f);
    run_filter(&b, 500, 0, 0, 0, ax * 9.81f, ay * 9.81f, az * 9.81f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, quaternion_get_roll(&a), quaternion_get_roll(&b));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, quaternion_get_pitch(&a), quaternion_get_pitch(&b));
}

void test_accel_cannot_correct_the_yaw(void) {
    /* The gravity has no information about the yaw */
    Quaternion q = quat_from_euler(0.0f, 0.0f, 40.0f);
    run_filter(&q, 1000, 0, 0, 0, 0, 0, 1.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 40.0f, quaternion_get_yaw(&q));
}

void test_accel_correction_is_slow_compared_to_the_gyro(void) {
    /* BETA = 0.08: in 0.1 s the accelerometer can only move a few degrees,
     * so short vibrations do not change the attitude much. */
    float ax, ay, az;
    Quaternion q;
    quaternion_init(&q);
    gravity(30.0f, 0.0f, &ax, &ay, &az);
    run_filter(&q, 10, 0, 0, 0, ax, ay, az, 0.01f);
    TEST_ASSERT_TRUE(quaternion_get_roll(&q) < 2.0f);
}

void test_accel_removes_the_gyro_drift(void) {
    /* A constant gyro bias of 0.5 deg/s on roll: without the accelerometer
     * the error grows forever, with it the error stays small. */
    Quaternion q;
    quaternion_init(&q);
    run_filter(&q, 6000, RAD(0.5f), 0, 0, 0, 0, 1.0f, 0.01f);   /* 60 s */
    TEST_ASSERT_TRUE_MESSAGE(fabsf(quaternion_get_roll(&q)) < 5.0f,
                             "with the accelerometer the drift must stay bounded");
}

void test_accel_tiny_reading_is_ignored(void) {
    Quaternion q;
    quaternion_init(&q);
    run_filter(&q, 100, 0, 0, 0, 1e-5f, 0, 0, 0.01f);   /* |a|^2 < 1e-6 */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, q.w);
}

/* ------------------------------------------------------------------ */
/*                              NORM                                  */
/* ------------------------------------------------------------------ */
void test_norm_is_1_after_every_update(void) {
    Quaternion q;
    quaternion_init(&q);
    for (int i = 0; i < 5000; i++) {
        quaternion_update(&q, sinf(i * 0.01f), cosf(i * 0.013f), 0.3f,
                          0.1f * sinf(i * 0.02f), 0.1f, 1.0f, 0.005f);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, norm(&q));
    }
}

void test_norm_not_normalised_input_is_normalised(void) {
    Quaternion q = { 2.0f, 0.0f, 0.0f, 0.0f };
    quaternion_update(&q, 0, 0, 0, 0, 0, 1.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, norm(&q));
}

void test_norm_outputs_are_never_nan(void) {
    Quaternion q;
    quaternion_init(&q);
    run_filter(&q, 1000, 10.0f, -10.0f, 10.0f, 0.0f, 5.0f, -3.0f, 0.02f);
    TEST_ASSERT_FALSE(isnan(q.w) || isnan(q.x) || isnan(q.y) || isnan(q.z));
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_helpers_inv_sqrt_values);
    RUN_TEST(test_helpers_inv_sqrt_zero_and_negative_are_zero);
    RUN_TEST(test_helpers_wrap_angle_inside_range_is_not_changed);
    RUN_TEST(test_helpers_wrap_angle_outside_range);

    RUN_TEST(test_euler_init_is_identity);
    RUN_TEST(test_euler_identity_is_zero_angles);
    RUN_TEST(test_euler_round_trip_on_a_grid);
    RUN_TEST(test_euler_angles_are_in_degrees);
    RUN_TEST(test_euler_pitch_90_does_not_give_nan);
    RUN_TEST(test_euler_pitch_minus_90_does_not_give_nan);
    RUN_TEST(test_euler_angles_are_always_in_range);

    RUN_TEST(test_gyro_no_rotation_keeps_identity);
    RUN_TEST(test_gyro_yaw_90_dps_for_1_s_is_90_deg);
    RUN_TEST(test_gyro_roll_30_dps_for_1_s_is_30_deg);
    RUN_TEST(test_gyro_pitch_minus_20_dps_for_1_s_is_minus_20_deg);
    RUN_TEST(test_gyro_result_does_not_depend_on_the_step);
    RUN_TEST(test_gyro_zero_dt_does_not_change_the_quaternion);

    RUN_TEST(test_accel_level_reading_keeps_identity);
    RUN_TEST(test_accel_converges_to_roll_30);
    RUN_TEST(test_accel_converges_to_pitch_minus_20);
    RUN_TEST(test_accel_units_do_not_matter);
    RUN_TEST(test_accel_cannot_correct_the_yaw);
    RUN_TEST(test_accel_correction_is_slow_compared_to_the_gyro);
    RUN_TEST(test_accel_removes_the_gyro_drift);
    RUN_TEST(test_accel_tiny_reading_is_ignored);

    RUN_TEST(test_norm_is_1_after_every_update);
    RUN_TEST(test_norm_not_normalised_input_is_normalised);
    RUN_TEST(test_norm_outputs_are_never_nan);

    return UNITY_END();
}
