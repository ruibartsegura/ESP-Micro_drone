/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for main.c (host, no hardware). app_main() must only start
 *   the system: everything else is done by the system task.
 *
 *   The .c file is included directly. Another version can be tested with:
 *       make run T=main MODULE_SRC=path/to/main.c
 *
 * Functions:
 *   - system_start(): mock of system.c, counts the calls.
 *   - setUp() / tearDown(): reset the mocks before each test.
 *   - test_app_main_*: app_main().
 *   - main(): runs all the tests.
 */
#include "unity.h"
#include "test_helpers.h"
#include "mocks/mock_reset.h"

#ifndef MODULE_SRC
#define MODULE_SRC "../main/main.c"
#endif
#include MODULE_SRC

static int system_start_calls;
void system_start(void) { system_start_calls++; }

void setUp(void) {
    mock_reset_all();
    system_start_calls = 0;
}

void tearDown(void) {}

void test_app_main_starts_the_system_once(void) {
    app_main();
    TEST_ASSERT_EQUAL_INT(1, system_start_calls);
}

void test_app_main_does_not_create_other_tasks(void) {
    app_main();
    TEST_ASSERT_EQUAL_INT(0, mock_task_count);   /* system_start() is mocked */
}

void test_app_main_returns_without_blocking(void) {
    /* app_main() must return: the ESP-IDF main task is deleted after it */
    app_main();
    TEST_ASSERT_EQUAL_INT(0, mock_delay_calls);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_app_main_starts_the_system_once);
    RUN_TEST(test_app_main_does_not_create_other_tasks);
    RUN_TEST(test_app_main_returns_without_blocking);
    return UNITY_END();
}
