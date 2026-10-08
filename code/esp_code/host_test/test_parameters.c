/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for parameters.c (host, no hardware). They check the
 *   default values that come from menuconfig and the setters/getters
 *   used by ROS (drone/params topic).
 *
 *   Units: CONFIG_HOVERING_H is in cm (menuconfig), the code works in m.
 *
 *   The .c file is included directly. Another version can be tested with:
 *       make run T=parameters MODULE_SRC=path/to/parameters.c
 *
 *   Tests that fail with the current code have [BUG] in their comment.
 *
 * Functions:
 *   - setUp() / tearDown(): restore the default values before each test.
 *   - test_default_*: values after boot.
 *   - test_set_get_*: setters and getters.
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

/* Default values saved before any test changes them. */
static float boot_hov_h;
static float boot_vel_max;
static bool  boot_land_on_site;

void setUp(void) {
    mock_reset_all();
    hov_h = boot_hov_h;
    vel_max = boot_vel_max;
    land_on_site = boot_land_on_site;
}

void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                            DEFAULTS                                */
/* ------------------------------------------------------------------ */
/* [BUG] hov_h = CONFIG_HOVERING_H / 100 is an integer division: 50 / 100 = 0.
 * It must be CONFIG_HOVERING_H / 100.0f = 0.5 m. */
void test_default_hovering_height_is_menuconfig_value_in_metres(void) {
    TEST_ASSERT_FLOAT_WITHIN(EPS, CONFIG_HOVERING_H / 100.0f, get_hovering_h());
}

/* [BUG] Same integer division: CONFIG_VEL_MAX / 100 = 2 / 100 = 0, so the
 * default maximum velocity is 0 and the drone could not move. */
void test_default_max_velocity_is_not_zero(void) {
    TEST_ASSERT_TRUE_MESSAGE(get_max_velocity() > 0.0f,
        "with CONFIG_VEL_MAX > 0 the default max velocity must be > 0");
}

void test_default_land_on_site_is_menuconfig_value(void) {
    TEST_ASSERT_EQUAL(CONFIG_LAND_ON_SITE ? true : false, get_land_on_site());
}

/* ------------------------------------------------------------------ */
/*                         SETTERS / GETTERS                          */
/* ------------------------------------------------------------------ */
void test_set_get_hovering_height(void) {
    set_hovering_h(1.25f);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.25f, get_hovering_h());
}

void test_set_get_max_velocity(void) {
    set_max_velocity(0.75f);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.75f, get_max_velocity());
}

void test_set_get_land_on_site_true_and_false(void) {
    set_land_on_site(false);
    TEST_ASSERT_FALSE(get_land_on_site());
    set_land_on_site(true);
    TEST_ASSERT_TRUE(get_land_on_site());
}

void test_set_get_parameters_are_independent(void) {
    set_hovering_h(2.0f);
    set_max_velocity(3.0f);
    set_land_on_site(false);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, get_hovering_h());
    TEST_ASSERT_FLOAT_WITHIN(EPS, 3.0f, get_max_velocity());
    TEST_ASSERT_FALSE(get_land_on_site());
}

void test_set_get_last_value_wins(void) {
    set_hovering_h(1.0f);
    set_hovering_h(2.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, get_hovering_h());
}

void test_set_get_global_variables_follow_the_setters(void) {
    /* parameters.h exports hov_h, vel_max and land_on_site: they must be
     * the same values as the getters. */
    set_hovering_h(0.8f);
    set_max_velocity(1.1f);
    set_land_on_site(false);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.8f, hov_h);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.1f, vel_max);
    TEST_ASSERT_FALSE(land_on_site);
}

int main(void) {
    boot_hov_h = hov_h;
    boot_vel_max = vel_max;
    boot_land_on_site = land_on_site;

    UNITY_BEGIN();

    RUN_TEST(test_default_hovering_height_is_menuconfig_value_in_metres);
    RUN_TEST(test_default_max_velocity_is_not_zero);
    RUN_TEST(test_default_land_on_site_is_menuconfig_value);

    RUN_TEST(test_set_get_hovering_height);
    RUN_TEST(test_set_get_max_velocity);
    RUN_TEST(test_set_get_land_on_site_true_and_false);
    RUN_TEST(test_set_get_parameters_are_independent);
    RUN_TEST(test_set_get_last_value_wins);
    RUN_TEST(test_set_get_global_variables_follow_the_setters);

    return UNITY_END();
}
