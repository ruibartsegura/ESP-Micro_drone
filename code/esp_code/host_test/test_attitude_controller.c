/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for attitude_controller.c (host, no hardware). They replace
 *   the old tests of components/flight_control/test, that used the old API
 *   (get_imu_data(), set_motor_speed(id, speed)). They cover:
 *     1. Complementary filter.
 *     2. Attitude estimation: get_roll_pitch() (gyro + accelerometer).
 *     3. Outer loop: cmd_vel_2_RP() (velocity -> target roll / pitch).
 *     4. Inner loop: angular rate -> motor mix (roll / pitch / yaw).
 *     5. Height: collective thrust and check_h_reached().
 *     6. Task and init.
 *
 *   The real state.c is linked (the controller reads the IMU and the
 *   height from the global state). get_attitude() (system.c) and
 *   set_motor_speed() (motors.c) are mocked.
 *
 *   Units (same as the drivers): acceleration in g (Z = +1 g at rest),
 *   angular velocity in deg/s, time in ns, angles of the estimator in
 *   degrees, output of cmd_vel_2_RP() in radians, cmd_vel in m/s and rad/s,
 *   height in m.
 *
 *   Motor mix (X frame), with the commands h, r, p, y:
 *     m0 = h + r - p - y      m1 = h - r - p + y
 *     m2 = h + r + p + y      m3 = h - r + p - y
 *   decompose() is the exact inverse of this mix.
 *
 *   The .c file is included directly, so the tests can see its macros
 *   (KP, MAX_ANGLE, ...), functions and globals. Another version can be
 *   tested with:
 *       make run T=attitude_controller MODULE_SRC=path/to/attitude_controller.c
 *
 *   Tests that fail with the current code have [BUG] in their comment.
 *
 * Functions:
 *   - get_attitude(), set_motor_speed(): mocks of system.c and motors.c.
 *   - decompose(): splits the 4 motor values into h, r, p, y.
 *   - set_tilt(), step(): helpers to prepare the IMU and run the controller.
 *   - setUp() / tearDown(): reset the mocks, the controller and the state.
 *   - test_comp_*, test_est_*, test_ext_*, test_int_*, test_h_*, test_task_*.
 *   - main(): runs all the tests.
 */
#include "unity.h"
#include "test_helpers.h"
#include "mocks/mock_reset.h"

#ifndef MODULE_SRC
#define MODULE_SRC "../components/flight_control/attitude_controller.c"
#endif
#include MODULE_SRC

/* Save the physical constants and remove the one letter macros (A, C, g, m). */
static const float K_RHO = rho;
static const float K_A   = A;
static const float K_CD  = C;
static const float K_G   = g;
static const float K_M   = m;
#undef A
#undef C
#undef g
#undef m
#undef rho

/* ------------------------------------------------------------------ */
/*                               MOCKS                                */
/* ------------------------------------------------------------------ */
static ATTITUDE_TARGET mock_target;
static double          motors[N_MOTORS];
static int             motor_calls;

ATTITUDE_TARGET get_attitude() { return mock_target; }

void set_motor_speed(double power[N_MOTORS]) {
    memcpy(motors, power, sizeof motors);
    motor_calls++;
}

/* ------------------------------------------------------------------ */
/*                              HELPERS                               */
/* ------------------------------------------------------------------ */
#define DT_NS 15000000LL   /* 15 ms, period of the task */
#define BASE  ((double)throttle_base)

typedef struct { double h, r, p, y; } Mix;

static Mix decompose(void) {
    Mix c;
    c.h = ( motors[0] + motors[1] + motors[2] + motors[3]) / 4.0;
    c.r = ( motors[0] - motors[1] + motors[2] - motors[3]) / 4.0;
    c.p = (-motors[0] - motors[1] + motors[2] + motors[3]) / 4.0;
    c.y = (-motors[0] + motors[1] + motors[2] - motors[3]) / 4.0;
    return c;
}

static int64_t t_now;

/* Accelerometer (in g) of a drone with this roll and pitch (degrees). */
static void set_tilt(float roll_deg, float pitch_deg) {
    set_acc_lin(-sinf(RAD(pitch_deg)),
                cosf(RAD(pitch_deg)) * sinf(RAD(roll_deg)),
                cosf(RAD(pitch_deg)) * cosf(RAD(roll_deg)));
}

/* One control cycle 15 ms later. */
static void step(void) {
    t_now += DT_NS;
    set_time_imu(t_now);
    control_attitude();
}

/* One control cycle with the same IMU timestamp: dt = 0, no gyro integration. */
static void step_same_time(void) {
    set_time_imu(last_rpy.t_stamp);
    control_attitude();
}

void setUp(void) {
    mock_reset_all();
    state_init();
    memset(&mock_target, 0, sizeof mock_target);
    memset(motors, 0, sizeof motors);
    motor_calls = 0;
    memset(&last_rpy, 0, sizeof last_rpy);
    err_h = 0.0f;
    t_now = 1000000000LL;   /* 1 s */
    last_rpy.t_stamp = t_now;
    set_time_imu(t_now);
    set_tilt(0, 0);         /* level, +1 g on Z */
}

void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                       1. COMPLEMENTARY FILTER                      */
/* ------------------------------------------------------------------ */
void test_comp_alpha_1_returns_the_first(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.0f, complementary_filter(3.0f, 7.0f, 1.0f));
}

void test_comp_alpha_0_returns_the_second(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 7.0f, complementary_filter(3.0f, 7.0f, 0.0f));
}

void test_comp_blend_value(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.98f * 10.0f + 0.02f * 20.0f, complementary_filter(10.0f, 20.0f, ALPHA));
}

void test_comp_alpha_trusts_the_gyro(void) {
    TEST_ASSERT_TRUE(ALPHA > 0.9 && ALPHA < 1.0);
}

/* ------------------------------------------------------------------ */
/*                       2. ATTITUDE ESTIMATION                       */
/* ------------------------------------------------------------------ */
void test_est_level_and_still_is_zero(void) {
    float r, p;
    t_now += DT_NS; set_time_imu(t_now);
    get_roll_pitch(&r, &p);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, r);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, p);
}

void test_est_accel_converges_to_the_roll(void) {
    float r = 0, p = 0;
    set_tilt(20.0f, 0.0f);
    for (int i = 0; i < 500; i++) { t_now += DT_NS; set_time_imu(t_now); get_roll_pitch(&r, &p); }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, r);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, p);
}

void test_est_accel_converges_to_the_pitch(void) {
    float r = 0, p = 0;
    set_tilt(0.0f, -15.0f);
    for (int i = 0; i < 500; i++) { t_now += DT_NS; set_time_imu(t_now); get_roll_pitch(&r, &p); }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -15.0f, p);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, r);
}

void test_est_one_step_of_the_accel_is_2_percent(void) {
    float r, p;
    set_tilt(10.0f, 0.0f);
    t_now += DT_NS; set_time_imu(t_now);
    get_roll_pitch(&r, &p);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, (1.0f - ALPHA) * 10.0f, r);
}

void test_est_gyro_is_integrated_with_the_timestamps(void) {
    float r, p;
    set_vel_ang(10.0f, -20.0f, 0.0f);   /* deg/s */
    t_now += 10000000LL;                /* 10 ms */
    set_time_imu(t_now);
    get_roll_pitch(&r, &p);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, ALPHA * 0.1f, r);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, ALPHA * -0.2f, p);
}

void test_est_gyro_1_s_at_30_dps_is_about_30_deg(void) {
    float r = 0, p = 0;
    /* tilt and gyro agree: the drone rotates at 30 deg/s */
    for (int i = 1; i <= 100; i++) {
        set_tilt(30.0f * i / 100.0f, 0.0f);
        set_vel_ang(30.0f, 0.0f, 0.0f);
        t_now += 10000000LL;
        set_time_imu(t_now);
        get_roll_pitch(&r, &p);
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 30.0f, r);
}

void test_est_saves_the_estimate_and_the_time(void) {
    float r, p;
    set_tilt(10.0f, 5.0f);
    t_now += DT_NS; set_time_imu(t_now);
    get_roll_pitch(&r, &p);
    TEST_ASSERT_EQUAL_FLOAT(r, last_rpy.roll);
    TEST_ASSERT_EQUAL_FLOAT(p, last_rpy.pitch);
    TEST_ASSERT_EQUAL_INT64(t_now, last_rpy.t_stamp);
}

void test_est_first_sample_long_dt_is_ignored(void) {
    /* after boot last_rpy.t_stamp = 0: dt would be huge */
    float r, p;
    last_rpy.t_stamp = 0;
    set_vel_ang(100.0f, 0, 0);
    set_time_imu(5000000000LL);
    get_roll_pitch(&r, &p);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, r);
}

void test_est_zero_dt_does_not_integrate(void) {
    float r, p;
    set_vel_ang(100.0f, 100.0f, 0);
    set_time_imu(last_rpy.t_stamp);
    get_roll_pitch(&r, &p);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, r);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, p);
}

void test_est_negative_dt_is_ignored(void) {
    float r, p;
    set_vel_ang(100.0f, 0, 0);
    set_time_imu(last_rpy.t_stamp - DT_NS);
    get_roll_pitch(&r, &p);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, r);
}

void test_est_accel_units_do_not_matter(void) {
    /* In simulation the acceleration arrives in m/s^2: atan2 gives the same angle */
    float r1, p1, r2, p2;
    set_acc_lin(-0.2f, 0.3f, 0.93f);
    t_now += DT_NS; set_time_imu(t_now);
    get_roll_pitch(&r1, &p1);
    memset(&last_rpy, 0, sizeof last_rpy);
    last_rpy.t_stamp = t_now;
    set_acc_lin(-0.2f * 9.81f, 0.3f * 9.81f, 0.93f * 9.81f);
    t_now += DT_NS; set_time_imu(t_now);
    get_roll_pitch(&r2, &p2);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, r1, r2);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, p1, p2);
}

/* ------------------------------------------------------------------ */
/*                   3. OUTER LOOP: cmd_vel -> roll/pitch             */
/* ------------------------------------------------------------------ */
static geometry_msgs__msg__Twist vel(double vx, double vy) {
    geometry_msgs__msg__Twist t = { 0 };
    t.linear.x = vx;
    t.linear.y = vy;
    return t;
}

/* Drag model: tan(angle) = Cd * rho * A * v^2 / (2 m g) */
static float drag_angle(float v) {
    return atan2f(K_CD * K_RHO * K_A * v * v, 2.0f * K_M * K_G);
}

void test_ext_zero_velocity_gives_zero_angles(void) {
    float r = 9, p = 9;
    cmd_vel_2_RP(&r, &p, vel(0, 0));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, r);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, p);
}

void test_ext_matches_the_drag_model(void) {
    float r, p;
    cmd_vel_2_RP(&r, &p, vel(2.0, 0));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, drag_angle(2.0f), p);
}

void test_ext_forward_velocity_gives_positive_pitch_only(void) {
    float r, p;
    cmd_vel_2_RP(&r, &p, vel(1.5, 0));
    TEST_ASSERT_TRUE(p > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, r);
}

void test_ext_lateral_velocity_gives_positive_roll_only(void) {
    float r, p;
    cmd_vel_2_RP(&r, &p, vel(0, 1.5));
    TEST_ASSERT_TRUE(r > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, p);
}

/* [BUG] The sign of the pitch comes from vy and the sign of the roll from
 * vx (sign_x and sign_y are swapped): going backwards (vx < 0, vy = 0)
 * gives the same pitch as going forwards. */
void test_ext_backward_velocity_gives_negative_pitch(void) {
    float r, p;
    cmd_vel_2_RP(&r, &p, vel(-1.5, 0));
    TEST_ASSERT_TRUE_MESSAGE(p < 0.0f, "backwards must tilt the other way");
}

/* [BUG] Same as above for the roll: moving left (vy < 0) gives a positive roll. */
void test_ext_left_velocity_gives_negative_roll(void) {
    float r, p;
    cmd_vel_2_RP(&r, &p, vel(0, -1.5));
    TEST_ASSERT_TRUE_MESSAGE(r < 0.0f, "left must tilt the other way");
}

/* The output must be odd: f(-v) = -f(v) (with both components negative
 * the swapped signs of the [BUG] above cancel out, so this one passes). */
void test_ext_is_odd_symmetric(void) {
    for (double v = 0.25; v <= 4.0; v += 0.25) {
        float r1, p1, r2, p2;
        cmd_vel_2_RP(&r1, &p1, vel(v, v * 0.5));
        cmd_vel_2_RP(&r2, &p2, vel(-v, -v * 0.5));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, -r1, r2);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, -p1, p2);
    }
}

void test_ext_grows_with_the_speed(void) {
    float prev = -1.0f;
    for (double v = 0.0; v <= 5.0; v += 0.25) {
        float r, p;
        cmd_vel_2_RP(&r, &p, vel(v, 0));
        TEST_ASSERT_TRUE(p >= prev);
        prev = p;
    }
}

void test_ext_diagonal_gives_equal_roll_and_pitch(void) {
    float r, p;
    cmd_vel_2_RP(&r, &p, vel(1.0, 1.0));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, r, p);
}

void test_ext_is_limited_to_max_angle(void) {
    float r, p;
    cmd_vel_2_RP(&r, &p, vel(50.0, 50.0));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, MAX_ANGLE, r);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, MAX_ANGLE, p);
}

void test_ext_never_over_max_angle(void) {
    for (double vx = -30; vx <= 30; vx += 1.5)
        for (double vy = -30; vy <= 30; vy += 1.5) {
            float r, p;
            cmd_vel_2_RP(&r, &p, vel(vx, vy));
            TEST_ASSERT_TRUE(fabsf(r) <= MAX_ANGLE + 1e-6f);
            TEST_ASSERT_TRUE(fabsf(p) <= MAX_ANGLE + 1e-6f);
        }
}

void test_ext_max_angle_is_30_deg(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, (float)DEG(MAX_ANGLE));
}

/* ------------------------------------------------------------------ */
/*                    4. INNER LOOP AND MOTOR MIX                     */
/* ------------------------------------------------------------------ */
void test_int_one_motor_command_per_cycle(void) {
    step();
    step();
    TEST_ASSERT_EQUAL_INT(2, motor_calls);
}

void test_int_outputs_are_finite(void) {
    set_vel_ang(1000.0f, -1000.0f, 500.0f);
    set_acc_lin(0, 0, 0);
    mock_target.cmd_vel = vel(3.0, -2.0);
    mock_target.h = 2.0f;
    step();
    for (int i = 0; i < N_MOTORS; i++) TEST_ASSERT_FALSE(isnan(motors[i]) || isinf(motors[i]));
}

void test_int_level_and_still_all_motors_are_equal(void) {
    step();
    for (int i = 1; i < N_MOTORS; i++) TEST_ASSERT_DOUBLE_WITHIN(1e-9, motors[0], motors[i]);
}

/* [BUG] throttle_base (2387, "Min throttle to hover") is never used: at the
 * target height the collective thrust is KP * 0 = 0 and the drone falls. */
void test_int_at_target_height_motors_get_the_hover_throttle(void) {
    set_h(0.5f);
    mock_target.h = 0.5f;
    step();
    for (int i = 0; i < N_MOTORS; i++)
        TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1.0, BASE, motors[i], "hover needs throttle_base on every motor");
}

void test_int_roll_rate_error_only_moves_the_roll_channel(void) {
    set_vel_ang(-10.0f, 0, 0);   /* the drone rolls at -10 deg/s, it should be 0 */
    step_same_time();
    Mix c = decompose();
    TEST_ASSERT_DOUBLE_WITHIN(1e-3, KP_RATE * 10.0, c.r);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, c.p);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, c.y);
}

void test_int_pitch_rate_error_only_moves_the_pitch_channel(void) {
    set_vel_ang(0, 8.0f, 0);
    step_same_time();
    Mix c = decompose();
    TEST_ASSERT_DOUBLE_WITHIN(1e-3, KP_RATE * -8.0, c.p);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, c.r);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, c.y);
}

/* A huge error must not take all the throttle: each roll/pitch/yaw term
 * is limited to MAX_POW_RPY so the motors keep the hover thrust. */
void test_int_attitude_power_is_limited(void) {
    set_vel_ang(-1000.0f, 1000.0f, 1000.0f);
    step_same_time();
    Mix c = decompose();
    TEST_ASSERT_DOUBLE_WITHIN(1e-3, MAX_POW_RPY, c.r);
    TEST_ASSERT_DOUBLE_WITHIN(1e-3, -MAX_POW_RPY, c.p);
    TEST_ASSERT_DOUBLE_WITHIN(1e-3, -MAX_POW_RPY, c.y);
}

void test_int_yaw_follows_cmd_vel_angular_z(void) {
    mock_target.cmd_vel.angular.z = 0.5;   /* rad/s, small so it is not limited by MAX_POW_RPY */
    step_same_time();
    Mix c = decompose();
    TEST_ASSERT_DOUBLE_WITHIN(0.1, KP_RATE * DEG(0.5), c.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, c.r);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, c.p);
}

void test_int_yaw_at_the_target_rate_gives_zero_yaw(void) {
    mock_target.cmd_vel.angular.z = 0.5;
    set_vel_ang(0, 0, (float)DEG(0.5));
    step_same_time();
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, decompose().y);
}

void test_int_response_is_proportional_to_the_error(void) {
    set_vel_ang(-5.0f, 0, 0);
    step_same_time();
    double r1 = decompose().r;
    set_vel_ang(-10.0f, 0, 0);
    step_same_time();
    double r2 = decompose().r;
    TEST_ASSERT_DOUBLE_WITHIN(1e-3, 2.0 * r1, r2);
}

void test_int_mix_does_not_change_the_collective_thrust(void) {
    step_same_time();
    double h0 = decompose().h;
    set_vel_ang(-20.0f, 15.0f, 10.0f);
    step_same_time();
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, h0, decompose().h);
}

void test_int_mix_is_x_frame(void) {
    /* opposite motors (0-3, 1-2) turn the same way: same sign of yaw */
    mock_target.cmd_vel.angular.z = 1.0;
    step_same_time();
    TEST_ASSERT_TRUE((motors[0] - motors[1]) * (motors[3] - motors[2]) > 0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, motors[0], motors[3]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, motors[1], motors[2]);
}

void test_int_tilt_is_opposed(void) {
    /* rolled +10 deg and still: the controller must roll back (r < 0) */
    last_rpy.roll = 10.0f;
    set_tilt(10.0f, 0.0f);
    step_same_time();
    TEST_ASSERT_TRUE(decompose().r < 0.0);
}

void test_int_pitch_tilt_is_opposed(void) {
    last_rpy.pitch = -10.0f;
    set_tilt(0.0f, -10.0f);
    step_same_time();
    TEST_ASSERT_TRUE(decompose().p > 0.0);
}

void test_int_forward_command_moves_the_pitch_channel(void) {
    mock_target.cmd_vel = vel(2.0, 0);
    step_same_time();
    Mix c = decompose();
    TEST_ASSERT_TRUE(c.p > 0.0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, c.r);
}

void test_int_no_error_when_the_tilt_is_the_target(void) {
    /* target pitch for 2 m/s, and the drone already has that pitch */
    float tr, tp;
    cmd_vel_2_RP(&tr, &tp, vel(2.0, 0));
    mock_target.cmd_vel = vel(2.0, 0);
    last_rpy.pitch = (float)DEG(tp);
    set_tilt(0.0f, (float)DEG(tp));
    step_same_time();
    TEST_ASSERT_DOUBLE_WITHIN(0.5, 0.0, decompose().p);
}

/* ------------------------------------------------------------------ */
/*                              5. HEIGHT                             */
/* ------------------------------------------------------------------ */
void test_h_below_target_more_thrust(void) {
    mock_target.h = 1.0f;
    set_h(1.0f);
    step();
    double at = decompose().h;
    set_h(0.5f);
    step();
    TEST_ASSERT_TRUE(decompose().h > at);
}

void test_h_above_target_less_thrust(void) {
    mock_target.h = 1.0f;
    set_h(1.0f);
    step();
    double at = decompose().h;
    set_h(1.5f);
    step();
    TEST_ASSERT_TRUE(decompose().h < at);
}

void test_h_thrust_is_proportional_to_the_error(void) {
    mock_target.h = 1.0f;
    set_h(1.0f);
    step();
    double at = decompose().h;
    set_h(0.8f);
    step();
    double d1 = decompose().h - at;
    set_h(0.6f);
    step();
    double d2 = decompose().h - at;
    TEST_ASSERT_DOUBLE_WITHIN(1e-3, 2.0 * d1, d2);
}

void test_h_error_is_saved(void) {
    mock_target.h = 1.0f;
    set_h(0.7f);
    step();
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.3f, err_h);
}

void test_h_reached_at_the_target(void) {
    mock_target.h = 1.0f;
    set_h(1.0f);
    step();
    TEST_ASSERT_TRUE(check_h_reached());
}

void test_h_reached_inside_1_cm(void) {
    mock_target.h = 1.0f;
    set_h(0.995f);
    step();
    TEST_ASSERT_TRUE(check_h_reached());
}

void test_h_not_reached_outside_1_cm(void) {
    mock_target.h = 1.0f;
    set_h(0.97f);
    step();
    TEST_ASSERT_FALSE(check_h_reached());
    set_h(1.03f);
    step();
    TEST_ASSERT_FALSE(check_h_reached());
}

void test_h_not_reached_far_away(void) {
    mock_target.h = 2.0f;
    set_h(0.0f);
    step();
    TEST_ASSERT_FALSE(check_h_reached());
}

/* ------------------------------------------------------------------ */
/*                           6. TASK / INIT                           */
/* ------------------------------------------------------------------ */
void test_task_init_creates_the_task_with_the_menuconfig_values(void) {
    init_attitude_controller();
    const mock_task_t *t = mock_find_task("attitude_task");
    TEST_ASSERT_NOT_NULL(t);
    TEST_ASSERT_EQUAL_UINT32(CONFIG_ATTITUDE_TASK_STACK, t->stack);
    TEST_ASSERT_EQUAL_UINT(CONFIG_ATTITUDE_TASK_PRIO, t->prio);
}

/* [BUG] Every other module ignores a second init (is_init). Here a second
 * call creates a second controller task that also drives the motors. */
void test_task_init_twice_creates_one_task(void) {
    init_attitude_controller();
    init_attitude_controller();
    TEST_ASSERT_EQUAL_INT(1, mock_task_count);
}

void test_task_period_is_15_ms(void) {
    mock_run_task(attitude_task, NULL, 10);
    TEST_ASSERT_EQUAL_UINT64(10 * 15, mock_delay_total_ms);
}

void test_task_controls_every_cycle(void) {
    mock_run_task(attitude_task, NULL, 10);
    TEST_ASSERT_EQUAL_INT(10, motor_calls);
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_comp_alpha_1_returns_the_first);
    RUN_TEST(test_comp_alpha_0_returns_the_second);
    RUN_TEST(test_comp_blend_value);
    RUN_TEST(test_comp_alpha_trusts_the_gyro);

    RUN_TEST(test_est_level_and_still_is_zero);
    RUN_TEST(test_est_accel_converges_to_the_roll);
    RUN_TEST(test_est_accel_converges_to_the_pitch);
    RUN_TEST(test_est_one_step_of_the_accel_is_2_percent);
    RUN_TEST(test_est_gyro_is_integrated_with_the_timestamps);
    RUN_TEST(test_est_gyro_1_s_at_30_dps_is_about_30_deg);
    RUN_TEST(test_est_saves_the_estimate_and_the_time);
    RUN_TEST(test_est_first_sample_long_dt_is_ignored);
    RUN_TEST(test_est_zero_dt_does_not_integrate);
    RUN_TEST(test_est_negative_dt_is_ignored);
    RUN_TEST(test_est_accel_units_do_not_matter);

    RUN_TEST(test_ext_zero_velocity_gives_zero_angles);
    RUN_TEST(test_ext_matches_the_drag_model);
    RUN_TEST(test_ext_forward_velocity_gives_positive_pitch_only);
    RUN_TEST(test_ext_lateral_velocity_gives_positive_roll_only);
    RUN_TEST(test_ext_backward_velocity_gives_negative_pitch);
    RUN_TEST(test_ext_left_velocity_gives_negative_roll);
    RUN_TEST(test_ext_is_odd_symmetric);
    RUN_TEST(test_ext_grows_with_the_speed);
    RUN_TEST(test_ext_diagonal_gives_equal_roll_and_pitch);
    RUN_TEST(test_ext_is_limited_to_max_angle);
    RUN_TEST(test_ext_never_over_max_angle);
    RUN_TEST(test_ext_max_angle_is_30_deg);

    RUN_TEST(test_int_one_motor_command_per_cycle);
    RUN_TEST(test_int_outputs_are_finite);
    RUN_TEST(test_int_level_and_still_all_motors_are_equal);
    RUN_TEST(test_int_at_target_height_motors_get_the_hover_throttle);
    RUN_TEST(test_int_roll_rate_error_only_moves_the_roll_channel);
    RUN_TEST(test_int_pitch_rate_error_only_moves_the_pitch_channel);
    RUN_TEST(test_int_attitude_power_is_limited);
    RUN_TEST(test_int_yaw_follows_cmd_vel_angular_z);
    RUN_TEST(test_int_yaw_at_the_target_rate_gives_zero_yaw);
    RUN_TEST(test_int_response_is_proportional_to_the_error);
    RUN_TEST(test_int_mix_does_not_change_the_collective_thrust);
    RUN_TEST(test_int_mix_is_x_frame);
    RUN_TEST(test_int_tilt_is_opposed);
    RUN_TEST(test_int_pitch_tilt_is_opposed);
    RUN_TEST(test_int_forward_command_moves_the_pitch_channel);
    RUN_TEST(test_int_no_error_when_the_tilt_is_the_target);

    RUN_TEST(test_h_below_target_more_thrust);
    RUN_TEST(test_h_above_target_less_thrust);
    RUN_TEST(test_h_thrust_is_proportional_to_the_error);
    RUN_TEST(test_h_error_is_saved);
    RUN_TEST(test_h_reached_at_the_target);
    RUN_TEST(test_h_reached_inside_1_cm);
    RUN_TEST(test_h_not_reached_outside_1_cm);
    RUN_TEST(test_h_not_reached_far_away);

    RUN_TEST(test_task_init_creates_the_task_with_the_menuconfig_values);
    RUN_TEST(test_task_init_twice_creates_one_task);
    RUN_TEST(test_task_period_is_15_ms);
    RUN_TEST(test_task_controls_every_cycle);

    return UNITY_END();
}
