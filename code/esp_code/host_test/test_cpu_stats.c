/**
 * Made by Rui B.S.
 * Date: 14/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for cpu_stats.c (host, no hardware). Built with
 *   CONFIG_CPU_STATS_ON (see the Makefile). They check the period of the
 *   tasks (task_period_tick(), period_take()), the run time of the previous
 *   window (run_before()), the log task and init_cpu_stats().
 *   uxTaskGetSystemState() is mocked here with a fake list of tasks.
 *
 *   The .c file is included directly. Another version can be tested with:
 *       make run T=cpu_stats MODULE_SRC=path/to/cpu_stats.c
 *
 * Functions:
 *   - uxTaskGetSystemState(): mock, returns fake_tasks.
 *   - setUp() / tearDown(): reset the mocks and the module.
 *   - test_period_*, test_run_*, test_task_*, test_init_*.
 *   - main(): runs all the tests.
 */
#include "unity.h"
#include "test_helpers.h"
#include "mocks/mock_reset.h"

#ifndef MODULE_SRC
#define MODULE_SRC "../components/diagnostics/cpu_stats.c"
#endif
#include MODULE_SRC

/* ------------------------------------------------------------------ */
/*                               MOCKS                                */
/* ------------------------------------------------------------------ */
static TaskStatus_t fake_tasks[3];
static uint32_t     fake_total;
static int          system_state_calls;

UBaseType_t uxTaskGetSystemState(TaskStatus_t *array, UBaseType_t size,
                                 configRUN_TIME_COUNTER_TYPE *total) {
    system_state_calls++;
    /* Every call is a new window: 2 s of total time, 0.5 s for each task */
    fake_total += 2000000;
    for (int i = 0; i < 3; i++) {
        fake_tasks[i].ulRunTimeCounter += 500000;
    }
    if (size < 3) {
        return 0;
    }
    memcpy(array, fake_tasks, sizeof(fake_tasks));
    *total = fake_total;
    return 3;
}

void setUp(void) {
    mock_reset_all();
    is_init = false;
    n_periods = 0;
    memset(periods, 0, sizeof(periods));
    memset(prev, 0, sizeof(prev));
    memset(cur, 0, sizeof(cur));
    mock_time_us = 1000000;   /* 1 s */
    fake_total = 0;
    system_state_calls = 0;
    const char *names[3] = { "IDLE", "attitude_task", "imu_task" };
    for (int i = 0; i < 3; i++) {
        memset(&fake_tasks[i], 0, sizeof(fake_tasks[i]));
        fake_tasks[i].xHandle = (TaskHandle_t)(intptr_t)(i + 1);
        fake_tasks[i].pcTaskName = names[i];
    }
}

void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                      PERIOD OF THE TASKS                           */
/* ------------------------------------------------------------------ */
void test_period_first_tick_registers_and_does_not_count(void) {
    task_period_t p = TASK_PERIOD("a", 10);
    task_period_tick(&p);
    TEST_ASSERT_TRUE(p.registered);
    TEST_ASSERT_EQUAL_INT(1, n_periods);
    TEST_ASSERT_EQUAL_PTR(&p, periods[0]);
    TEST_ASSERT_EQUAL_UINT32(0, p.n);
}

void test_period_registers_only_once(void) {
    task_period_t p = TASK_PERIOD("a", 10);
    for (int i = 0; i < 5; i++) {
        task_period_tick(&p);
    }
    TEST_ASSERT_EQUAL_INT(1, n_periods);
}

void test_period_measures_the_time_between_ticks(void) {
    task_period_t p = TASK_PERIOD("a", 10);
    task_period_tick(&p);
    mock_time_us += 10000;
    task_period_tick(&p);
    mock_time_us += 12000;
    task_period_tick(&p);
    TEST_ASSERT_EQUAL_UINT32(2, p.n);
    TEST_ASSERT_EQUAL_INT64(22000, p.sum_us);
    TEST_ASSERT_EQUAL_INT64(12000, p.max_us);
}

void test_period_late_is_more_than_2_periods(void) {
    task_period_t p = TASK_PERIOD("a", 10);
    task_period_tick(&p);
    mock_time_us += 20000;   /* exactly 2 periods: not late */
    task_period_tick(&p);
    mock_time_us += 20001;
    task_period_tick(&p);
    mock_time_us += 100000;
    task_period_tick(&p);
    TEST_ASSERT_EQUAL_UINT32(2, p.late);
}

void test_period_take_copies_and_starts_a_new_window(void) {
    task_period_t p = TASK_PERIOD("a", 10);
    task_period_tick(&p);
    mock_time_us += 50000;
    task_period_tick(&p);
    task_period_t c = period_take(&p);
    TEST_ASSERT_EQUAL_UINT32(1, c.n);
    TEST_ASSERT_EQUAL_INT64(50000, c.max_us);
    TEST_ASSERT_EQUAL_UINT32(1, c.late);
    TEST_ASSERT_EQUAL_UINT32(0, p.n);
    TEST_ASSERT_EQUAL_UINT32(0, p.late);
    TEST_ASSERT_EQUAL_INT64(0, p.sum_us);
    TEST_ASSERT_EQUAL_INT64(0, p.max_us);
    /* The next period is measured from the last tick, not from the take */
    mock_time_us += 10000;
    task_period_tick(&p);
    TEST_ASSERT_EQUAL_INT64(10000, p.max_us);
}

void test_period_list_is_limited_to_max_periods(void) {
    static task_period_t p[MAX_PERIODS + 3];
    for (int i = 0; i < MAX_PERIODS + 3; i++) {
        p[i] = (task_period_t)TASK_PERIOD("x", 10);
        task_period_tick(&p[i]);
    }
    TEST_ASSERT_EQUAL_INT(MAX_PERIODS, n_periods);
    TEST_ASSERT_TRUE(p[MAX_PERIODS + 2].registered);
}

void test_period_uses_the_critical_section(void) {
    task_period_t p = TASK_PERIOD("a", 10);
    task_period_tick(&p);
    task_period_tick(&p);
    period_take(&p);
    TEST_ASSERT_EQUAL_INT(0, lock.locked);
}

/* ------------------------------------------------------------------ */
/*                       RUN TIME OF THE TASKS                        */
/* ------------------------------------------------------------------ */
void test_run_before_finds_the_task(void) {
    TaskStatus_t list[2] = { 0 };
    list[0].xHandle = (TaskHandle_t)1;
    list[0].ulRunTimeCounter = 111;
    list[1].xHandle = (TaskHandle_t)2;
    list[1].ulRunTimeCounter = 222;
    TEST_ASSERT_EQUAL_UINT32(222, run_before((TaskHandle_t)2, list, 2));
}

void test_run_before_of_a_new_task_is_0(void) {
    TaskStatus_t list[1] = { 0 };
    list[0].xHandle = (TaskHandle_t)1;
    list[0].ulRunTimeCounter = 111;
    TEST_ASSERT_EQUAL_UINT32(0, run_before((TaskHandle_t)9, list, 1));
}

/* ------------------------------------------------------------------ */
/*                          TASK AND INIT                             */
/* ------------------------------------------------------------------ */
void test_task_waits_the_window_of_the_menuconfig(void) {
    mock_run_task(cpu_stats_task, NULL, 3);
    TEST_ASSERT_EQUAL_UINT64(3 * CONFIG_CPU_STATS_PERIOD_MS, mock_delay_total_ms);
}

void test_task_reads_the_tasks_once_per_window(void) {
    mock_run_task(cpu_stats_task, NULL, 3);
    /* 1 reading before the loop + 1 per window (the 3rd delay stops it) */
    TEST_ASSERT_EQUAL_INT(3, system_state_calls);
}

void test_task_saves_the_window_as_the_previous_one(void) {
    mock_run_task(cpu_stats_task, NULL, 2);
    TEST_ASSERT_EQUAL_UINT32(fake_tasks[1].ulRunTimeCounter, prev[1].ulRunTimeCounter);
}

void test_task_starts_a_new_period_window(void) {
    task_period_t p = TASK_PERIOD("a", 10);
    task_period_tick(&p);
    mock_time_us += 10000;
    task_period_tick(&p);
    mock_run_task(cpu_stats_task, NULL, 2);
    TEST_ASSERT_EQUAL_UINT32(0, p.n);
}

void test_init_creates_the_task(void) {
    init_cpu_stats();
    const mock_task_t *t = mock_find_task("cpu_stats");
    TEST_ASSERT_NOT_NULL(t);
    TEST_ASSERT_EQUAL_UINT32(CPU_STATS_TASK_STACK, t->stack);
    TEST_ASSERT_EQUAL_UINT32(CPU_STATS_TASK_PRIO, t->prio);
}

void test_init_twice_creates_one_task(void) {
    init_cpu_stats();
    init_cpu_stats();
    TEST_ASSERT_EQUAL_INT(1, mock_task_count);
}

/* It has to write the log even if the drone tasks take all the CPU. */
void test_init_priority_is_over_the_drone_tasks(void) {
    TEST_ASSERT_TRUE(CPU_STATS_TASK_PRIO > CONFIG_IMU_TASK_PRIO);
    TEST_ASSERT_TRUE(CPU_STATS_TASK_PRIO > CONFIG_MICRO_ROS_TASK_PRIO);
    TEST_ASSERT_TRUE(CPU_STATS_TASK_PRIO > CONFIG_ATTITUDE_TASK_PRIO);
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_period_first_tick_registers_and_does_not_count);
    RUN_TEST(test_period_registers_only_once);
    RUN_TEST(test_period_measures_the_time_between_ticks);
    RUN_TEST(test_period_late_is_more_than_2_periods);
    RUN_TEST(test_period_take_copies_and_starts_a_new_window);
    RUN_TEST(test_period_list_is_limited_to_max_periods);
    RUN_TEST(test_period_uses_the_critical_section);

    RUN_TEST(test_run_before_finds_the_task);
    RUN_TEST(test_run_before_of_a_new_task_is_0);

    RUN_TEST(test_task_waits_the_window_of_the_menuconfig);
    RUN_TEST(test_task_reads_the_tasks_once_per_window);
    RUN_TEST(test_task_saves_the_window_as_the_previous_one);
    RUN_TEST(test_task_starts_a_new_period_window);

    RUN_TEST(test_init_creates_the_task);
    RUN_TEST(test_init_twice_creates_one_task);
    RUN_TEST(test_init_priority_is_over_the_drone_tasks);

    return UNITY_END();
}
