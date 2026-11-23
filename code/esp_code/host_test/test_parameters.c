/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for parameters.c (host, no hardware). They check the
 *   default values that come from menuconfig, the setters/getters, the
 *   table of names and ranges (param_info[]) and params_update(), used by
 *   ROS (drone/params) and by the serial console (tuning.c).
 *
 *   Units: menuconfig only has integers (CONFIG_VEL_MAX in cm/s, gains
 *   x1000...), parameters.c scales them.
 *
 *   The .c file is included directly. Another version can be tested with:
 *       make run T=parameters MODULE_SRC=path/to/parameters.c
 *
 *   Tests that fail with the current code have [BUG] in their comment.
 *
 * Functions:
 *   - setUp() / tearDown(): restore the default values before each test.
 *   - field(): pointer to a field of params_t by its param_info[] index.
 *   - test_default_*: values after boot.
 *   - test_set_get_*: setters and getters.
 *   - test_table_*: param_info[] and params_find().
 *   - test_update_*: params_update() and PARAM_KEEP.
 *   - test_lock_*: every access uses the critical section.
 *   - main(): runs all the tests.
 */
#include "unity.h"
#include "test_helpers.h"
#include "mocks/mock_reset.h"

#ifndef MODULE_SRC
#define MODULE_SRC "../components/global_data/parameters/parameters.c"
#endif
#include MODULE_SRC

#define EPS 1e-6f

static float *field(params_t *p, size_t i) {
    return (float *)((char *)p + param_info[i].offset);
}

void setUp(void) {
    mock_reset_all();
    params_init();
}

void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                            DEFAULTS                                */
/* ------------------------------------------------------------------ */
void test_default_max_velocity_is_menuconfig_value_in_m_s(void) {
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_VEL_MAX / 100.0f, get_max_velocity());
}

void test_default_max_velocity_is_not_zero(void) {
    TEST_ASSERT_TRUE_MESSAGE(get_max_velocity() > 0.0f,
        "with CONFIG_VEL_MAX > 0 the default max velocity must be > 0");
}

void test_default_gains_are_the_menuconfig_values(void) {
    GAINS k;
    get_gains(&k);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_KP_H / 1000.0f, k.kp_h);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_KD_H / 1000.0f, k.kd_h);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_KP_VEL / 1000.0f, k.kp_vel);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_KP_ANGLE / 1000.0f, k.kp_angle);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_KP_RATE / 1000.0f, k.kp_rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_KI_RATE / 1000.0f, k.ki_rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_KP_YAW_RATE / 1000.0f, k.kp_yaw_rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_MAX_POWER_RATE, k.max_rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_MAX_POWER_RP, k.max_pow_rp);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_THROTTLE_BASE, k.base);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_ALPHA / 100.0f, k.alpha);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_MAX_TILT_ANGLE / 100.0f, k.max_angle);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_MAX_POWER_YAW, k.max_pow_yaw);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_PITCH_DEVIATION / 10.0f, k.pitch_deviation);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_ROLL_DEVIATION / 10.0f, k.roll_deviation);
}

void test_default_k_drag_is_the_menuconfig_value(void) {
    odom_params_t o;
    get_odom_params(&o);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_K_DRAG / 1000.0f, o.k_drag);
}

/* The defaults must pass their own validation, or ROS could never change
 * one field (params_update() checks the whole struct it writes). */
void test_default_values_are_inside_their_ranges(void) {
    params_t d;
    get_default_params(&d);
    for (size_t i = 0; i < n_param_info; i++) {
        float v = *field(&d, i);
        TEST_ASSERT_TRUE_MESSAGE(v >= param_info[i].min && v <= param_info[i].max,
                                 param_info[i].name);
    }
}

void test_default_init_restores_everything(void) {
    set_max_velocity(1.3f);
    GAINS k;
    get_gains(&k);
    k.kp_rate = 1.0f;
    set_gains(&k);
    odom_params_t o = { .k_drag = 0.5f };
    set_odom_params(&o);

    params_init();

    params_t now, d;
    get_all_params(&now);
    get_default_params(&d);
    TEST_ASSERT_EQUAL_MEMORY(&d, &now, sizeof d);
}

/* ------------------------------------------------------------------ */
/*                         SETTERS / GETTERS                          */
/* ------------------------------------------------------------------ */
void test_set_get_max_velocity(void) {
    set_max_velocity(0.75f);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.75f, get_max_velocity());
}

void test_set_get_gains_round_trip(void) {
    GAINS in, out;
    get_gains(&in);
    in.kp_h = 1.0f;
    in.alpha = 0.5f;
    in.base = 3.0f;
    set_gains(&in);
    get_gains(&out);
    TEST_ASSERT_EQUAL_MEMORY(&in, &out, sizeof in);
}

void test_set_get_odom_round_trip(void) {
    odom_params_t in = { .k_drag = 0.3f }, out;
    set_odom_params(&in);
    get_odom_params(&out);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.3f, out.k_drag);
}

void test_set_get_all_params_is_coherent_with_the_groups(void) {
    set_max_velocity(3.0f);
    params_t p;
    get_all_params(&p);
    GAINS k;
    get_gains(&k);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 3.0f, p.flight.vel_max);
    TEST_ASSERT_EQUAL_MEMORY(&k, &p.gains, sizeof k);

    p.gains.kp_vel = 0.2f;
    set_all_params(&p);
    get_gains(&k);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.2f, k.kp_vel);
}

/* ------------------------------------------------------------------ */
/*                              TABLE                                 */
/* ------------------------------------------------------------------ */
/* One entry for each float of params_t, each one in a different place. */
void test_table_covers_every_field_once(void) {
    TEST_ASSERT_EQUAL_size_t(sizeof(params_t) / sizeof(float), n_param_info);
    bool used[sizeof(params_t) / sizeof(float)] = { false };
    for (size_t i = 0; i < n_param_info; i++) {
        size_t idx = param_info[i].offset / sizeof(float);
        TEST_ASSERT_FALSE_MESSAGE(used[idx], param_info[i].name);
        used[idx] = true;
        TEST_ASSERT_TRUE(param_info[i].min <= param_info[i].max);
        TEST_ASSERT_TRUE_MESSAGE(param_info[i].min >= 0.0f,
                                 "a negative value would be PARAM_KEEP");
    }
}

void test_table_find_by_name_any_case(void) {
    const param_info_t *i = params_find("KP_Rate");
    TEST_ASSERT_NOT_NULL(i);
    TEST_ASSERT_EQUAL_size_t(offsetof(params_t, gains.kp_rate), i->offset);
    TEST_ASSERT_NULL(params_find("kp_foo"));
}

/* ------------------------------------------------------------------ */
/*                             UPDATE                                 */
/* ------------------------------------------------------------------ */
void test_update_all_keep_changes_nothing(void) {
    params_t before, after, in;
    get_all_params(&before);
    params_keep_all(&in);
    TEST_ASSERT_NULL(params_update(&in));
    get_all_params(&after);
    TEST_ASSERT_EQUAL_MEMORY(&before, &after, sizeof before);
}

void test_update_only_changes_the_filled_fields(void) {
    params_t before, after, in;
    get_all_params(&before);
    params_keep_all(&in);
    in.gains.kp_rate = 9.0f;
    in.odom.k_drag = 0.2f;
    TEST_ASSERT_NULL(params_update(&in));
    get_all_params(&after);
    before.gains.kp_rate = 9.0f;
    before.odom.k_drag = 0.2f;
    TEST_ASSERT_EQUAL_MEMORY(&before, &after, sizeof before);
}

/* Any negative value is "keep", not only -1. */
void test_update_any_negative_value_keeps_the_field(void) {
    params_t in;
    params_keep_all(&in);
    in.gains.kd_h = -0.5f;
    in.gains.base = -1000.0f;
    TEST_ASSERT_NULL(params_update(&in));
    GAINS k;
    get_gains(&k);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_KD_H / 1000.0f, k.kd_h);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_THROTTLE_BASE, k.base);
}

void test_update_zero_is_a_valid_value(void) {
    params_t in;
    params_keep_all(&in);
    in.gains.ki_rate = 0.0f;
    TEST_ASSERT_NULL(params_update(&in));
    GAINS k;
    get_gains(&k);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, k.ki_rate);
}

/* One value out of range: nothing changes and it returns its name. */
void test_update_out_of_range_rejects_everything(void) {
    params_t before, after, in;
    get_all_params(&before);
    params_keep_all(&in);
    in.flight.vel_max = 1.0f;     /* valid */
    in.gains.alpha = 1.5f;        /* > 1 */
    TEST_ASSERT_EQUAL_STRING("alpha", params_update(&in));
    get_all_params(&after);
    TEST_ASSERT_EQUAL_MEMORY(&before, &after, sizeof before);
}

void test_update_nan_and_inf_are_rejected(void) {
    params_t in;
    params_keep_all(&in);
    in.gains.kp_h = NAN;
    TEST_ASSERT_EQUAL_STRING("kp_h", params_update(&in));
    params_keep_all(&in);
    in.gains.kp_h = INFINITY;
    TEST_ASSERT_EQUAL_STRING("kp_h", params_update(&in));
    params_keep_all(&in);
    in.gains.kp_h = -INFINITY;
    TEST_ASSERT_EQUAL_STRING("kp_h", params_update(&in));
    GAINS k;
    get_gains(&k);
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_KP_H / 1000.0f, k.kp_h);
}

/* k_drag divides in the odometry: 0 is not allowed. */
void test_update_k_drag_zero_is_rejected(void) {
    params_t in;
    params_keep_all(&in);
    in.odom.k_drag = 0.0f;
    TEST_ASSERT_EQUAL_STRING("k_drag", params_update(&in));
}

void test_update_limits_are_inclusive(void) {
    for (size_t i = 0; i < n_param_info; i++) {
        params_t in;
        params_keep_all(&in);
        *field(&in, i) = param_info[i].min;
        TEST_ASSERT_NULL_MESSAGE(params_update(&in), param_info[i].name);
        params_keep_all(&in);
        *field(&in, i) = param_info[i].max;
        TEST_ASSERT_NULL_MESSAGE(params_update(&in), param_info[i].name);
    }
}

/* ------------------------------------------------------------------ */
/*                              LOCK                                  */
/* ------------------------------------------------------------------ */
/* Every access leaves the critical section unlocked. */
void test_lock_is_released_after_every_call(void) {
    params_t p;
    GAINS k;
    odom_params_t o;
    get_all_params(&p);
    set_all_params(&p);
    get_gains(&k);
    set_gains(&k);
    get_odom_params(&o);
    set_odom_params(&o);
    set_max_velocity(get_max_velocity());
    params_keep_all(&p);
    params_update(&p);
    params_init();
    TEST_ASSERT_EQUAL_INT(0, p_lock.locked);
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_default_max_velocity_is_menuconfig_value_in_m_s);
    RUN_TEST(test_default_max_velocity_is_not_zero);
    RUN_TEST(test_default_gains_are_the_menuconfig_values);
    RUN_TEST(test_default_k_drag_is_the_menuconfig_value);
    RUN_TEST(test_default_values_are_inside_their_ranges);
    RUN_TEST(test_default_init_restores_everything);

    RUN_TEST(test_set_get_max_velocity);
    RUN_TEST(test_set_get_gains_round_trip);
    RUN_TEST(test_set_get_odom_round_trip);
    RUN_TEST(test_set_get_all_params_is_coherent_with_the_groups);

    RUN_TEST(test_table_covers_every_field_once);
    RUN_TEST(test_table_find_by_name_any_case);

    RUN_TEST(test_update_all_keep_changes_nothing);
    RUN_TEST(test_update_only_changes_the_filled_fields);
    RUN_TEST(test_update_any_negative_value_keeps_the_field);
    RUN_TEST(test_update_zero_is_a_valid_value);
    RUN_TEST(test_update_out_of_range_rejects_everything);
    RUN_TEST(test_update_nan_and_inf_are_rejected);
    RUN_TEST(test_update_k_drag_zero_is_rejected);
    RUN_TEST(test_update_limits_are_inclusive);

    RUN_TEST(test_lock_is_released_after_every_call);

    return UNITY_END();
}
