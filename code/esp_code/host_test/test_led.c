/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for led.c (host, no hardware). The GPIO driver is mocked
 *   and records every pin level, so the tests check which pin each LED
 *   uses, the init, the cached status and the blink test.
 *
 *   The .c file is included directly, so the tests can see its static
 *   variables (is_init, led_status). Another version can be tested with:
 *       make run T=led MODULE_SRC=path/to/led.c
 *
 *   Tests that fail with the current code have [BUG] in their comment.
 *
 * Functions:
 *   - setUp() / tearDown(): reset the mocks and the module before each test.
 *   - test_init_*: initialisation of the GPIOs.
 *   - test_on_off_*: led_on(), led_off(), all_on(), all_off().
 *   - test_led_test_*: the blink test.
 *   - main(): runs all the tests.
 */
#include "unity.h"
#include "test_helpers.h"
#include "mocks/mock_reset.h"

#ifndef MODULE_SRC
#define MODULE_SRC "../components/drivers/leds/led.c"
#endif
#include MODULE_SRC

static const led_t ALL_LEDS[N_LEDS] = { LED_ESP, LED_BLUE, LED_RED, LED_GREEN };
static const int   ALL_PINS[N_LEDS] = { CONFIG_LED_PIN_ESP, CONFIG_LED_PIN_BLUE,
                                        CONFIG_LED_PIN_RED, CONFIG_LED_PIN_GREEN };

/* Put the module back as it is after boot. */
static void reset_module(void) {
    is_init = false;
    led_status[LED_ESP]   = ESP_LED_STATUS;
    led_status[LED_RED]   = RED_LED_STATUS;
    led_status[LED_GREEN] = GREEN_LED_STATUS;
    led_status[LED_BLUE]  = BLUE_LED_STATUS;
}

/* Init and forget the GPIO calls done by the init. */
static void init_and_clear(void) {
    led_init();
    mock_reset_all();
}

void setUp(void) {
    mock_reset_all();
    reset_module();
}

void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                              INIT                                  */
/* ------------------------------------------------------------------ */
void test_init_configures_the_4_pins_as_outputs(void) {
    led_init();
    TEST_ASSERT_EQUAL_INT(N_LEDS, mock_gpio_config_calls);
    for (int i = 0; i < N_LEDS; i++)
        TEST_ASSERT_EQUAL_INT_MESSAGE(GPIO_MODE_OUTPUT, mock_gpio_mode[ALL_PINS[i]],
                                      "every LED pin must be an output");
}

void test_init_uses_the_pins_from_menuconfig(void) {
    TEST_ASSERT_EQUAL_INT(CONFIG_LED_PIN_ESP,   led_pin[LED_ESP]);
    TEST_ASSERT_EQUAL_INT(CONFIG_LED_PIN_BLUE,  led_pin[LED_BLUE]);
    TEST_ASSERT_EQUAL_INT(CONFIG_LED_PIN_RED,   led_pin[LED_RED]);
    TEST_ASSERT_EQUAL_INT(CONFIG_LED_PIN_GREEN, led_pin[LED_GREEN]);
}

void test_init_turns_every_led_off(void) {
    led_init();
    for (int i = 0; i < N_LEDS; i++)
        TEST_ASSERT_EQUAL_INT(LED_OFF, mock_gpio_level[ALL_PINS[i]]);
}

void test_init_does_not_touch_other_pins(void) {
    led_init();
    for (int p = 0; p < MOCK_GPIO_PINS; p++) {
        bool is_led = false;
        for (int i = 0; i < N_LEDS; i++) is_led |= (p == ALL_PINS[i]);
        if (!is_led) TEST_ASSERT_EQUAL_INT_MESSAGE(-1, mock_gpio_mode[p], "only the LED pins must be configured");
    }
}

void test_init_twice_configures_only_once(void) {
    led_init();
    led_init();
    TEST_ASSERT_EQUAL_INT(N_LEDS, mock_gpio_config_calls);
}

/* ------------------------------------------------------------------ */
/*                           ON / OFF                                 */
/* ------------------------------------------------------------------ */
void test_on_off_each_led_uses_its_own_pin(void) {
    init_and_clear();
    for (int i = 0; i < N_LEDS; i++) {
        led_on(ALL_LEDS[i]);
        TEST_ASSERT_EQUAL_INT(LED_ON, mock_gpio_level[ALL_PINS[i]]);
        /* the others stay as they were */
        for (int j = i + 1; j < N_LEDS; j++)
            TEST_ASSERT_EQUAL_INT(-1, mock_gpio_level[ALL_PINS[j]]);
    }
}

void test_on_off_led_off_turns_the_pin_off(void) {
    init_and_clear();
    led_on(LED_RED);
    led_off(LED_RED);
    TEST_ASSERT_EQUAL_INT(LED_OFF, mock_gpio_level[CONFIG_LED_PIN_RED]);
}

void test_on_off_led_on_twice_writes_the_pin_once(void) {
    init_and_clear();
    led_on(LED_GREEN);
    led_on(LED_GREEN);
    TEST_ASSERT_EQUAL_INT(1, mock_gpio_set_calls[CONFIG_LED_PIN_GREEN]);
}

void test_on_off_led_off_when_already_off_does_not_write(void) {
    init_and_clear();
    led_off(LED_BLUE);
    TEST_ASSERT_EQUAL_INT(0, mock_gpio_set_calls[CONFIG_LED_PIN_BLUE]);
}

void test_on_off_status_follows_the_led(void) {
    init_and_clear();
    led_on(LED_ESP);
    TEST_ASSERT_EQUAL_INT(LED_ON, led_status[LED_ESP]);
    led_off(LED_ESP);
    TEST_ASSERT_EQUAL_INT(LED_OFF, led_status[LED_ESP]);
}

void test_on_off_all_on_turns_every_led_on(void) {
    init_and_clear();
    all_on();
    for (int i = 0; i < N_LEDS; i++) {
        TEST_ASSERT_EQUAL_INT(LED_ON, mock_gpio_level[ALL_PINS[i]]);
        TEST_ASSERT_EQUAL_INT(LED_ON, led_status[ALL_LEDS[i]]);
    }
}

void test_on_off_all_off_turns_every_led_off(void) {
    init_and_clear();
    all_on();
    all_off();
    for (int i = 0; i < N_LEDS; i++) {
        TEST_ASSERT_EQUAL_INT(LED_OFF, mock_gpio_level[ALL_PINS[i]]);
        TEST_ASSERT_EQUAL_INT(LED_OFF, led_status[ALL_LEDS[i]]);
    }
}

void test_on_off_after_all_on_led_on_does_not_write_again(void) {
    init_and_clear();
    all_on();
    int calls = mock_gpio_set_calls[CONFIG_LED_PIN_RED];
    led_on(LED_RED);
    TEST_ASSERT_EQUAL_INT(calls, mock_gpio_set_calls[CONFIG_LED_PIN_RED]);
}

void test_on_off_after_all_off_led_on_works_again(void) {
    init_and_clear();
    all_on();
    all_off();
    led_on(LED_BLUE);
    TEST_ASSERT_EQUAL_INT(LED_ON, mock_gpio_level[CONFIG_LED_PIN_BLUE]);
}

/* ------------------------------------------------------------------ */
/*                            LED TEST                                */
/* ------------------------------------------------------------------ */
void test_led_test_before_init_returns_false(void) {
    TEST_ASSERT_FALSE(led_test());
    TEST_ASSERT_EQUAL_INT(0, mock_gpio_log_len);
}

void test_led_test_after_init_returns_true(void) {
    led_init();
    TEST_ASSERT_TRUE(led_test());
}

void test_led_test_blinks_every_led_on_off_on(void) {
    init_and_clear();
    led_test();
    for (int i = 0; i < N_LEDS; i++) {
        int seq[8], n = 0;
        for (int e = 0; e < mock_gpio_log_len && n < 8; e++)
            if (mock_gpio_log[e].pin == ALL_PINS[i]) seq[n++] = mock_gpio_log[e].level;
        TEST_ASSERT_TRUE_MESSAGE(n >= 3, "every LED must blink");
        TEST_ASSERT_EQUAL_INT(1, seq[0]);
        TEST_ASSERT_EQUAL_INT(0, seq[1]);
        TEST_ASSERT_EQUAL_INT(1, seq[2]);
    }
}

void test_led_test_tests_the_leds_one_by_one(void) {
    init_and_clear();
    led_test();
    /* The first 3 writes are for the first LED, the next 3 for the second... */
    for (int i = 0; i < N_LEDS; i++)
        for (int k = 0; k < 3; k++)
            TEST_ASSERT_EQUAL_INT(led_pin[i], mock_gpio_log[i * 3 + k].pin);
}

void test_led_test_ends_with_every_led_off(void) {
    init_and_clear();
    led_test();
    for (int i = 0; i < N_LEDS; i++) {
        TEST_ASSERT_EQUAL_INT(LED_OFF, mock_gpio_level[ALL_PINS[i]]);
        TEST_ASSERT_EQUAL_INT(LED_OFF, led_status[ALL_LEDS[i]]);
    }
}

void test_led_test_takes_750ms_per_led(void) {
    init_and_clear();
    led_test();
    TEST_ASSERT_EQUAL_UINT64(N_LEDS * 750, mock_delay_total_ms);
}

/* [BUG] led_test() turns every LED off at the end, also the ones that were
 * on before. The system calls it in CHECKING, after micro-ROS can have
 * turned on LED_ESP ("connected to the agent"), so that information is lost. */
void test_led_test_keeps_the_leds_that_were_on(void) {
    init_and_clear();
    led_on(LED_ESP);
    led_test();
    TEST_ASSERT_EQUAL_INT_MESSAGE(LED_ON, mock_gpio_level[CONFIG_LED_PIN_ESP],
        "LED_ESP was on before led_test() and must be on after it");
    TEST_ASSERT_EQUAL_INT(LED_ON, led_status[LED_ESP]);
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_init_configures_the_4_pins_as_outputs);
    RUN_TEST(test_init_uses_the_pins_from_menuconfig);
    RUN_TEST(test_init_turns_every_led_off);
    RUN_TEST(test_init_does_not_touch_other_pins);
    RUN_TEST(test_init_twice_configures_only_once);

    RUN_TEST(test_on_off_each_led_uses_its_own_pin);
    RUN_TEST(test_on_off_led_off_turns_the_pin_off);
    RUN_TEST(test_on_off_led_on_twice_writes_the_pin_once);
    RUN_TEST(test_on_off_led_off_when_already_off_does_not_write);
    RUN_TEST(test_on_off_status_follows_the_led);
    RUN_TEST(test_on_off_all_on_turns_every_led_on);
    RUN_TEST(test_on_off_all_off_turns_every_led_off);
    RUN_TEST(test_on_off_after_all_on_led_on_does_not_write_again);
    RUN_TEST(test_on_off_after_all_off_led_on_works_again);

    RUN_TEST(test_led_test_before_init_returns_false);
    RUN_TEST(test_led_test_after_init_returns_true);
    RUN_TEST(test_led_test_blinks_every_led_on_off_on);
    RUN_TEST(test_led_test_tests_the_leds_one_by_one);
    RUN_TEST(test_led_test_ends_with_every_led_off);
    RUN_TEST(test_led_test_takes_750ms_per_led);
    RUN_TEST(test_led_test_keeps_the_leds_that_were_on);

    return UNITY_END();
}
