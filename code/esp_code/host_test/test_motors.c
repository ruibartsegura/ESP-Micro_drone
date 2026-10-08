/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for motors.c (host, no hardware). The file is built twice:
 *     - motors_hw:  real drone, the motors are driven with LEDC (PWM).
 *                   The power is a percentage (0..100 %).
 *     - motors_sim: CONFIG_SIMULATION_ON, the power is published to ROS
 *                   with pub_motor_speed() (mocked here). Range 0..MAX_POWER.
 *   They check the arming, the limits of the power, the stop and the
 *   motor test.
 *
 *   The .c file is included directly. Another version can be tested with:
 *       make run T=motors_hw MODULE_SRC=path/to/motors.c
 *
 *   Tests that fail with the current code have [BUG] in their comment.
 *
 * Functions:
 *   - pub_motor_speed(): (simulation) mock that saves every published frame.
 *   - motor_out(): last value sent to a motor (duty in hw, power in sim).
 *   - setUp() / tearDown(): reset the mocks and the module before each test.
 *   - test_init_*: motors_init().
 *   - test_arm_*: arm_motors() / disarm_motors().
 *   - test_speed_*: set_motor_speed().
 *   - test_stop_*: motors_stop_all().
 *   - test_mtest_*: motors_test().
 *   - main(): runs all the tests.
 */
#include "unity.h"
#include "test_helpers.h"
#include "mocks/mock_reset.h"

#ifndef MODULE_SRC
#define MODULE_SRC "../components/drivers/motors/motors.c"
#endif
#include MODULE_SRC

#define MAX_DUTY ((1u << LEDC_DUTY_RES) - 1)   /* 1023 */

#ifdef CONFIG_SIMULATION_ON
/* ------------------------------------------------------------------ */
/*                  MOCK of pub_motor_speed (ros_coordinator)         */
/* ------------------------------------------------------------------ */
#define MAX_FRAMES 4096
static double frames[MAX_FRAMES][NUM_MOTORS];
static int    n_frames;
static void (*pub_hook)(void);

void pub_motor_speed(const double power[NUM_MOTORS]) {
    if (n_frames < MAX_FRAMES) memcpy(frames[n_frames], power, sizeof frames[0]);
    n_frames++;
    if (pub_hook) pub_hook();
}

static double motor_out(int i) {
    TEST_ASSERT_TRUE_MESSAGE(n_frames > 0, "nothing was published");
    return frames[(n_frames > MAX_FRAMES ? MAX_FRAMES : n_frames) - 1][i];
}
#define FULL     ((double)MAX_POWER)
#define HALF     (MAX_POWER / 2.0)
#else
static double motor_out(int i) { return (double)mock_ledc_out_duty[motor_channel[i]]; }
#define FULL     100.0
#define HALF     50.0
#endif

static void reset_module(void) {
    is_init = false;
    is_testing = false;
    drone_armed = false;
}

static void set_all(double v) {
    double p[N_MOTORS] = { v, v, v, v };
    set_motor_speed(p);
}

void setUp(void) {
    mock_reset_all();
    reset_module();
#ifdef CONFIG_SIMULATION_ON
    n_frames = 0;
    pub_hook = NULL;
#endif
    motors_init();
}

void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                               INIT                                 */
/* ------------------------------------------------------------------ */
void test_init_4_motors(void) {
    TEST_ASSERT_EQUAL_INT(4, N_MOTORS);
    TEST_ASSERT_EQUAL_INT(N_MOTORS, NUM_MOTORS);   /* motors.h and ros_coordinator.h must agree */
}

void test_init_twice_is_harmless(void) {
    int timers = mock_ledc_timer_config_calls, chans = mock_ledc_channel_config_calls;
    motors_init();
    TEST_ASSERT_EQUAL_INT(timers, mock_ledc_timer_config_calls);
    TEST_ASSERT_EQUAL_INT(chans, mock_ledc_channel_config_calls);
}

void test_init_starts_disarmed(void) {
    TEST_ASSERT_FALSE(drone_armed);
}

#ifdef CONFIG_SIMULATION_ON
void test_init_simulation_does_not_use_the_pwm(void) {
    TEST_ASSERT_EQUAL_INT(0, mock_ledc_timer_config_calls);
    TEST_ASSERT_EQUAL_INT(0, mock_ledc_channel_config_calls);
    TEST_ASSERT_TRUE(is_init);
}
#else
void test_init_one_pwm_timer_at_20khz_10_bits(void) {
    TEST_ASSERT_EQUAL_INT(1, mock_ledc_timer_config_calls);
    TEST_ASSERT_EQUAL_INT(LEDC_LOW_SPEED_MODE, mock_ledc_timer_conf.speed_mode);
    TEST_ASSERT_EQUAL_INT(LEDC_TIMER_0, mock_ledc_timer_conf.timer_num);
    TEST_ASSERT_EQUAL_INT(LEDC_TIMER_10_BIT, mock_ledc_timer_conf.duty_resolution);
    TEST_ASSERT_EQUAL_UINT32(20000, mock_ledc_timer_conf.freq_hz);
}

void test_init_one_channel_per_motor_on_the_menuconfig_pins(void) {
    const int pins[N_MOTORS] = { CONFIG_MOTOR01_PIN, CONFIG_MOTOR02_PIN, CONFIG_MOTOR03_PIN, CONFIG_MOTOR04_PIN };
    TEST_ASSERT_EQUAL_INT(N_MOTORS, mock_ledc_channel_config_calls);
    for (int i = 0; i < N_MOTORS; i++) {
        ledc_channel_t ch = motor_channel[i];
        TEST_ASSERT_TRUE(mock_ledc_channel_configured[ch]);
        TEST_ASSERT_EQUAL_INT(pins[i], mock_ledc_channel_conf[ch].gpio_num);
        TEST_ASSERT_EQUAL_INT(LEDC_TIMER_0, mock_ledc_channel_conf[ch].timer_sel);
    }
}

void test_init_channels_are_different(void) {
    for (int i = 0; i < N_MOTORS; i++)
        for (int j = i + 1; j < N_MOTORS; j++)
            TEST_ASSERT_NOT_EQUAL(motor_channel[i], motor_channel[j]);
}

void test_init_motors_start_stopped(void) {
    for (int i = 0; i < N_MOTORS; i++)
        TEST_ASSERT_EQUAL_UINT32(0, mock_ledc_channel_conf[motor_channel[i]].duty);
}
#endif

/* ------------------------------------------------------------------ */
/*                              ARMING                                */
/* ------------------------------------------------------------------ */
void test_arm_disarmed_motors_get_zero(void) {
    set_all(HALF);
    for (int i = 0; i < N_MOTORS; i++) TEST_ASSERT_EQUAL_DOUBLE(0.0, motor_out(i));
}

void test_arm_armed_motors_get_the_power(void) {
    arm_motors();
    set_all(HALF);
#ifdef CONFIG_SIMULATION_ON
    for (int i = 0; i < N_MOTORS; i++) TEST_ASSERT_EQUAL_DOUBLE(HALF, motor_out(i));
#else
    for (int i = 0; i < N_MOTORS; i++) TEST_ASSERT_EQUAL_UINT32(50 * MAX_DUTY / 100, (uint32_t)motor_out(i));
#endif
}

void test_arm_disarm_stops_the_power_again(void) {
    arm_motors();
    set_all(HALF);
    disarm_motors();
    set_all(HALF);
    for (int i = 0; i < N_MOTORS; i++) TEST_ASSERT_EQUAL_DOUBLE(0.0, motor_out(i));
}

/* ------------------------------------------------------------------ */
/*                           MOTOR SPEED                              */
/* ------------------------------------------------------------------ */
void test_speed_each_motor_gets_its_own_value(void) {
    arm_motors();
    double p[N_MOTORS] = { FULL * 0.1, FULL * 0.2, FULL * 0.3, FULL * 0.4 };
    set_motor_speed(p);
    for (int i = 0; i < N_MOTORS; i++) {
#ifdef CONFIG_SIMULATION_ON
        TEST_ASSERT_DOUBLE_WITHIN(1e-9, FULL * 0.1 * (i + 1), motor_out(i));
#else
        TEST_ASSERT_UINT32_WITHIN(1, (uint32_t)(MAX_DUTY * 0.1 * (i + 1)), (uint32_t)motor_out(i));
#endif
    }
}

void test_speed_over_the_maximum_is_limited(void) {
    arm_motors();
    set_all(FULL * 3);
#ifdef CONFIG_SIMULATION_ON
    for (int i = 0; i < N_MOTORS; i++) TEST_ASSERT_EQUAL_DOUBLE(FULL, motor_out(i));
#else
    for (int i = 0; i < N_MOTORS; i++) TEST_ASSERT_EQUAL_UINT32(MAX_DUTY, (uint32_t)motor_out(i));
#endif
}

void test_speed_maximum_is_full_power(void) {
    arm_motors();
    set_all(FULL);
#ifdef CONFIG_SIMULATION_ON
    for (int i = 0; i < N_MOTORS; i++) TEST_ASSERT_EQUAL_DOUBLE(FULL, motor_out(i));
#else
    for (int i = 0; i < N_MOTORS; i++) TEST_ASSERT_EQUAL_UINT32(MAX_DUTY, (uint32_t)motor_out(i));
#endif
}

void test_speed_negative_is_zero(void) {
    /* The controller can ask for negative values (more than the
     * collective thrust); a motor cannot spin backwards. */
    arm_motors();
    set_all(-FULL * 0.3);
    for (int i = 0; i < N_MOTORS; i++) TEST_ASSERT_EQUAL_DOUBLE(0.0, motor_out(i));
}

void test_speed_huge_value_does_not_wrap_around(void) {
    /* Converting 300 % to an 8-bit integer would give 44 %: it must stay full. */
    arm_motors();
    double p[N_MOTORS] = { 1e6, FULL * 3, FULL + 156.0, 1e9 };
    set_motor_speed(p);
#ifdef CONFIG_SIMULATION_ON
    for (int i = 0; i < N_MOTORS; i++) TEST_ASSERT_EQUAL_DOUBLE(FULL, motor_out(i));
#else
    for (int i = 0; i < N_MOTORS; i++) TEST_ASSERT_EQUAL_UINT32(MAX_DUTY, (uint32_t)motor_out(i));
#endif
}

/* [BUG] A NaN from the controller must never spin a motor. The limits use
 * "power > MAX" and "power < 0", and both are false for NaN, so the NaN is
 * sent to the motors. */
void test_speed_nan_is_zero(void) {
    arm_motors();
    set_all(NAN);
    for (int i = 0; i < N_MOTORS; i++) TEST_ASSERT_EQUAL_DOUBLE(0.0, motor_out(i));
}

void test_speed_every_call_updates_the_4_motors(void) {
    arm_motors();
    set_all(HALF);
#ifdef CONFIG_SIMULATION_ON
    TEST_ASSERT_EQUAL_INT(1, n_frames);
#else
    for (int i = 0; i < N_MOTORS; i++) {
        TEST_ASSERT_EQUAL_INT(1, mock_ledc_set_calls[motor_channel[i]]);
        TEST_ASSERT_EQUAL_INT(1, mock_ledc_update_calls[motor_channel[i]]);
    }
#endif
}

/* ------------------------------------------------------------------ */
/*                               STOP                                 */
/* ------------------------------------------------------------------ */
void test_stop_all_sets_every_motor_to_zero(void) {
    arm_motors();
    set_all(HALF);
    motors_stop_all();
    for (int i = 0; i < N_MOTORS; i++) TEST_ASSERT_EQUAL_DOUBLE(0.0, motor_out(i));
}

void test_stop_all_works_even_when_armed(void) {
    arm_motors();
    motors_stop_all();
    TEST_ASSERT_TRUE(drone_armed);   /* stop does not disarm ... */
    for (int i = 0; i < N_MOTORS; i++) TEST_ASSERT_EQUAL_DOUBLE(0.0, motor_out(i));   /* ... but stops */
}

/* ------------------------------------------------------------------ */
/*                            MOTOR TEST                              */
/* ------------------------------------------------------------------ */
void test_mtest_before_init_returns_false(void) {
    reset_module();
    TEST_ASSERT_FALSE(motors_test());
}

void test_mtest_after_init_returns_true(void) {
    TEST_ASSERT_TRUE(motors_test());
}

void test_mtest_ends_with_every_motor_stopped(void) {
    motors_test();
    for (int i = 0; i < N_MOTORS; i++) TEST_ASSERT_EQUAL_DOUBLE(0.0, motor_out(i));
}

void test_mtest_ends_with_testing_mode_off(void) {
    motors_test();
    TEST_ASSERT_FALSE(is_testing);
    arm_motors();
    set_all(HALF);
    TEST_ASSERT_TRUE(motor_out(0) > 0.0);   /* set_motor_speed works again */
}

#ifdef CONFIG_SIMULATION_ON
/* While the test runs, set_motor_speed() (the controller) is ignored. */
static int ctrl_frames_inside;
static void call_controller_inside_test(void) {
    if (!is_testing) return;
    int before = n_frames;
    pub_hook = NULL;
    double p[N_MOTORS] = { 1, 1, 1, 1 };
    set_motor_speed(p);
    if (n_frames != before) ctrl_frames_inside++;
    pub_hook = call_controller_inside_test;
}

void test_mtest_controller_is_ignored_during_the_test(void) {
    arm_motors();
    ctrl_frames_inside = 0;
    pub_hook = call_controller_inside_test;
    motors_test();
    pub_hook = NULL;
    TEST_ASSERT_EQUAL_INT(0, ctrl_frames_inside);
}

/* Fill the stack with zeros: this test only checks the order (the garbage
 * of the not initialised array is checked in test_mtest_only_the_motor_under_test_spins). */
__attribute__((noinline)) static void clean_stack(void) {
    volatile unsigned char z[16384];
    memset((void *)z, 0, sizeof z);
}

void test_mtest_motors_are_tested_in_order_0_to_3(void) {
    clean_stack();
    motors_test();
    int last_motor = -1;
    for (int f = 0; f < n_frames && f < MAX_FRAMES; f++)
        for (int i = 0; i < N_MOTORS; i++)
            if (frames[f][i] > 0.0 && frames[f][i] < 1e6) {
                TEST_ASSERT_TRUE_MESSAGE(i >= last_motor, "the motors must be tested in order");
                last_motor = i;
            }
    TEST_ASSERT_EQUAL_INT(N_MOTORS - 1, last_motor);
}

void test_mtest_ramp_goes_up_and_down(void) {
    clean_stack();
    motors_test();
    double max0 = 0.0;
    int i_max = 0;
    for (int f = 0; f < n_frames; f++)
        if (frames[f][0] > max0 && frames[f][0] < 1e6) { max0 = frames[f][0]; i_max = f; }
    TEST_ASSERT_TRUE(max0 > 0.0);
    TEST_ASSERT_TRUE(max0 <= TEST_POWER);
    TEST_ASSERT_TRUE_MESSAGE(i_max > 0, "the ramp must go up first");
    TEST_ASSERT_TRUE_MESSAGE(frames[i_max + 1][0] < max0, "and then down");
}

void test_mtest_power_never_over_the_maximum(void) {
    motors_test();
    for (int f = 0; f < n_frames; f++)
        for (int i = 0; i < N_MOTORS; i++)
            TEST_ASSERT_TRUE(frames[f][i] <= MAX_POWER);
}

/* [BUG] In motors_test() the array power[] is not initialised: while one
 * motor is tested, the other motors get garbage from the stack. Here the
 * stack is filled with garbage first, to make the problem visible. */
void test_mtest_only_the_motor_under_test_spins(void) {
    poison_stack();
    motors_test();
    for (int f = 0; f < n_frames && f < MAX_FRAMES; f++) {
        int spinning = 0;
        for (int i = 0; i < N_MOTORS; i++) spinning += (frames[f][i] != 0.0);
        TEST_ASSERT_TRUE_MESSAGE(spinning <= 1, "only one motor at a time can spin during the test");
    }
}
#endif

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_init_4_motors);
    RUN_TEST(test_init_twice_is_harmless);
    RUN_TEST(test_init_starts_disarmed);
#ifdef CONFIG_SIMULATION_ON
    RUN_TEST(test_init_simulation_does_not_use_the_pwm);
#else
    RUN_TEST(test_init_one_pwm_timer_at_20khz_10_bits);
    RUN_TEST(test_init_one_channel_per_motor_on_the_menuconfig_pins);
    RUN_TEST(test_init_channels_are_different);
    RUN_TEST(test_init_motors_start_stopped);
#endif

    RUN_TEST(test_arm_disarmed_motors_get_zero);
    RUN_TEST(test_arm_armed_motors_get_the_power);
    RUN_TEST(test_arm_disarm_stops_the_power_again);

    RUN_TEST(test_speed_each_motor_gets_its_own_value);
    RUN_TEST(test_speed_over_the_maximum_is_limited);
    RUN_TEST(test_speed_maximum_is_full_power);
    RUN_TEST(test_speed_negative_is_zero);
    RUN_TEST(test_speed_huge_value_does_not_wrap_around);
    RUN_TEST(test_speed_nan_is_zero);
    RUN_TEST(test_speed_every_call_updates_the_4_motors);

    RUN_TEST(test_stop_all_sets_every_motor_to_zero);
    RUN_TEST(test_stop_all_works_even_when_armed);

    RUN_TEST(test_mtest_before_init_returns_false);
    RUN_TEST(test_mtest_after_init_returns_true);
    RUN_TEST(test_mtest_ends_with_every_motor_stopped);
    RUN_TEST(test_mtest_ends_with_testing_mode_off);
#ifdef CONFIG_SIMULATION_ON
    RUN_TEST(test_mtest_controller_is_ignored_during_the_test);
    RUN_TEST(test_mtest_motors_are_tested_in_order_0_to_3);
    RUN_TEST(test_mtest_ramp_goes_up_and_down);
    RUN_TEST(test_mtest_power_never_over_the_maximum);
    RUN_TEST(test_mtest_only_the_motor_under_test_spins);
#endif

    return UNITY_END();
}
