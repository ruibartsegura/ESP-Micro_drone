/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for height.c (host, no hardware). The file is built twice:
 *     - height_hw:  real drone. A fake BMP180 is simulated on the mocked
 *                   I2C bus, so the tests check the bus configuration, the
 *                   init of the sensor and the altitude saved in the state.
 *     - height_sim: CONFIG_SIMULATION_ON. The sensor is not used (the
 *                   height comes from ROS), only the task is started.
 *   The real bmp180.c and state.c are linked.
 *
 *   The BMP180 and the MPU-6050 are on the same GY-87 board and share the
 *   I2C bus 0 (pins CONFIG_I2C0_PIN_SDA / SCL).
 *
 *   The .c file is included directly. Another version can be tested with:
 *       make run T=height_hw MODULE_SRC=path/to/height.c
 *
 *   Tests that fail with the current code have [BUG] in their comment.
 *
 * Functions:
 *   - fake_bmp_*(): (hardware) the simulated BMP180.
 *   - setUp() / tearDown(): reset the mocks, the module and the state.
 *   - test_init_*: height_init() and height_test().
 *   - test_data_*: get_height_data().
 *   - test_sample_*: (hardware) h_sample(), reading of the sensor.
 *   - test_task_*: the FreeRTOS task.
 *   - main(): runs all the tests.
 */
#include "unity.h"
#include "test_helpers.h"
#include "mocks/mock_reset.h"

#ifndef MODULE_SRC
#define MODULE_SRC "../components/drivers/height/height.c"
#endif
#include MODULE_SRC

#define BMP_ADDR BMP180_I2C_ADDR

#ifndef CONFIG_SIMULATION_ON
/* ------------------------------------------------------------------ */
/*                    FAKE BMP180 (datasheet example)                 */
/* ------------------------------------------------------------------ */
static int32_t fake_ut = 27898, fake_up = 23843;

static void fake_bmp_write_hook(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len) {
    if (addr != BMP_ADDR || reg != 0xF4 || len < 1) return;
    if (data[0] == 0x2E) {
        mock_i2c_regs[BMP_ADDR][0xF6] = (uint8_t)(fake_ut >> 8);
        mock_i2c_regs[BMP_ADDR][0xF7] = (uint8_t)fake_ut;
    } else if ((data[0] & 0x3F) == 0x34) {
        /* fake_up is the value for oss = 0; with more oversampling the
         * chip gives more resolution bits for the same pressure. */
        int oss = data[0] >> 6;
        uint32_t raw = ((uint32_t)fake_up << oss) << (8 - oss);
        mock_i2c_regs[BMP_ADDR][0xF6] = (uint8_t)(raw >> 16);
        mock_i2c_regs[BMP_ADDR][0xF7] = (uint8_t)(raw >> 8);
        mock_i2c_regs[BMP_ADDR][0xF8] = (uint8_t)raw;
    }
}

static void fake_bmp_reset(void) {
    static const int16_t cal[11] = { 408, -72, -14383, (int16_t)32741, (int16_t)32757,
                                     (int16_t)23153, 6190, 4, -32768, -8711, 2868 };
    mock_i2c_regs[BMP_ADDR][0xD0] = 0x55;
    for (int i = 0; i < 11; i++) mock_i2c_set_be16(BMP_ADDR, 0xAA + 2 * i, cal[i]);
    fake_ut = 27898;
    fake_up = 23843;
    mock_i2c_write_hook = fake_bmp_write_hook;
}

/* Altitude of the datasheet example pressure (69964 Pa) with the
 * reference 101325 Pa. */
static float expected_alt(void) {
    int32_t p;
    bmp180_read_pressure(&bmp, &p);
    return bmp180_pressure_to_altitude(p, 101325.0f);
}
#endif

static void reset_module(void) {
    is_init = false;
    memset(&latest_data, 0, sizeof latest_data);
}

void setUp(void) {
    mock_reset_all();
    reset_module();
    state_init();
#ifndef CONFIG_SIMULATION_ON
    fake_bmp_reset();
#endif
}

void tearDown(void) {
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, mock_critical_depth, "a critical section was not released");
}

/* ------------------------------------------------------------------ */
/*                               INIT                                 */
/* ------------------------------------------------------------------ */
void test_init_test_is_false_before_init(void) {
    TEST_ASSERT_FALSE(height_test());
}

void test_init_test_is_true_after_init(void) {
    height_init();
    TEST_ASSERT_TRUE(height_test());
}

void test_init_creates_the_height_task(void) {
    height_init();
    TEST_ASSERT_EQUAL_INT(1, mock_task_count);
    TEST_ASSERT_NOT_NULL(mock_find_task("bmp180_task"));
}

/* [BUG] The stack and priority of the task are fixed in the code (4096, 5)
 * instead of using the values of menuconfig ("tasks settings"), like every
 * other task: CONFIG_HEIGHT_TASK_STACK and CONFIG_HEIGHT_TASK_PRIO. */
void test_init_task_uses_the_menuconfig_stack_and_priority(void) {
    height_init();
    const mock_task_t *t = mock_find_task("bmp180_task");
    TEST_ASSERT_NOT_NULL(t);
    TEST_ASSERT_EQUAL_UINT32(CONFIG_HEIGHT_TASK_STACK, t->stack);
    TEST_ASSERT_EQUAL_UINT(CONFIG_HEIGHT_TASK_PRIO, t->prio);
}

void test_init_twice_creates_one_task(void) {
    height_init();
    height_init();
    TEST_ASSERT_EQUAL_INT(1, mock_task_count);
}

#ifdef CONFIG_SIMULATION_ON
void test_init_simulation_does_not_use_the_i2c_bus(void) {
    height_init();
    TEST_ASSERT_EQUAL_INT(0, mock_i2c_param_config_calls);
    TEST_ASSERT_EQUAL_INT(0, mock_i2c_driver_install_calls);
    TEST_ASSERT_EQUAL_INT(0, mock_i2c_transactions);
}
#else
void test_init_configures_i2c_bus_0_with_the_menuconfig_pins(void) {
    height_init();
    TEST_ASSERT_TRUE(mock_i2c_param_config_calls >= 1);
    TEST_ASSERT_EQUAL_INT(CONFIG_I2C0_PIN_SDA, mock_i2c_conf[I2C_NUM_0].sda_io_num);
    TEST_ASSERT_EQUAL_INT(CONFIG_I2C0_PIN_SCL, mock_i2c_conf[I2C_NUM_0].scl_io_num);
    TEST_ASSERT_EQUAL_INT(GPIO_PULLUP_ENABLE, mock_i2c_conf[I2C_NUM_0].sda_pullup_en);
    TEST_ASSERT_EQUAL_INT(GPIO_PULLUP_ENABLE, mock_i2c_conf[I2C_NUM_0].scl_pullup_en);
    TEST_ASSERT_EQUAL_UINT32(400000, mock_i2c_conf[I2C_NUM_0].master.clk_speed);
}

/* [BUG] height.h does "#define I2C_MODE_MASTER I2C_NUM_0" (it should be
 * I2C_MASTER_NUM). That macro replaces the ESP-IDF value I2C_MODE_MASTER (1)
 * by 0, which is I2C_MODE_SLAVE: the bus is configured as slave. */
void test_init_bus_is_configured_as_master(void) {
    height_init();
    TEST_ASSERT_EQUAL_INT_MESSAGE(MOCK_I2C_MODE_MASTER, mock_i2c_conf[I2C_NUM_0].mode,
                                  "i2c_param_config() must receive I2C_MODE_MASTER");
    TEST_ASSERT_EQUAL_INT(MOCK_I2C_MODE_MASTER, mock_i2c_installed_mode[I2C_NUM_0]);
}

void test_init_detects_the_bmp180(void) {
    height_init();
    TEST_ASSERT_TRUE(mock_i2c_log_len > 0);
    TEST_ASSERT_EQUAL_HEX8(BMP_ADDR, mock_i2c_log[0].addr);
    TEST_ASSERT_EQUAL_HEX8(0xD0, mock_i2c_log[0].reg);   /* chip id */
}

void test_init_no_error_check_failure_when_all_is_ok(void) {
    height_init();
    TEST_ASSERT_EQUAL_INT(0, mock_esp_error_check_fails);
}

/* [BUG] The BMP180 and the MPU-6050 share the I2C bus 0 and the IMU is
 * started first (system_init()). height_init() installs the driver again,
 * i2c_driver_install() fails because the port is already installed, and
 * ESP_ERROR_CHECK() resets the ESP32. */
void test_init_works_when_the_imu_already_installed_the_bus(void) {
    i2c_driver_install(I2C_NUM_0, I2C_MODE_SLAVE + 1, 0, 0, 0);   /* the IMU did it before */
    height_init();
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, mock_esp_error_check_fails,
        "a bus already installed by the IMU must not abort the program");
    TEST_ASSERT_TRUE(height_test());
}

/* [BUG] If the BMP180 does not answer, i2c_master_init() calls
 * vTaskDelete(NULL): it deletes the task that called height_init(), that is
 * the system task (state machine). The error must be returned instead. */
void test_init_missing_sensor_does_not_delete_the_calling_task(void) {
    mock_i2c_regs[BMP_ADDR][0xD0] = 0x00;   /* wrong chip id */
    height_init();
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, mock_task_delete_calls, "the system task must not be deleted");
}

void test_init_missing_sensor_reports_the_failure(void) {
    mock_i2c_regs[BMP_ADDR][0xD0] = 0x00;
    height_init();
    TEST_ASSERT_FALSE_MESSAGE(height_test(), "without sensor height_test() must fail");
}
#endif

/* ------------------------------------------------------------------ */
/*                          get_height_data                           */
/* ------------------------------------------------------------------ */
void test_data_null_is_invalid_arg(void) {
    height_init();
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, get_height_data(NULL));
}

void test_data_before_init_is_invalid_state(void) {
    geometry_msgs__msg__PoseStamped d;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, get_height_data(&d));
}

void test_data_after_init_is_ok(void) {
    geometry_msgs__msg__PoseStamped d;
    height_init();
    TEST_ASSERT_EQUAL_INT(ESP_OK, get_height_data(&d));
}

void test_data_uses_the_critical_section(void) {
    geometry_msgs__msg__PoseStamped d;
    height_init();
    int before = mock_critical_enter_count;
    get_height_data(&d);
    TEST_ASSERT_TRUE(mock_critical_enter_count > before);
}

#ifndef CONFIG_SIMULATION_ON
/* [BUG] h_sample() saves the altitude with set_h() but never updates
 * latest_data, so get_height_data() always returns 0. */
void test_data_returns_the_last_altitude(void) {
    geometry_msgs__msg__PoseStamped d;
    height_init();
    h_sample();
    get_height_data(&d);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expected_alt(), (float)d.pose.position.z);
}

/* ------------------------------------------------------------------ */
/*                       SENSOR READING (h_sample)                    */
/* ------------------------------------------------------------------ */
void test_sample_saves_the_altitude_in_the_state(void) {
    height_init();
    h_sample();
    vec3_t p;
    get_position(&p);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expected_alt(), p.z);
    TEST_ASSERT_TRUE(p.z > 2900.0f && p.z < 3100.0f);   /* 69964 Pa = ~3 km */
}

void test_sample_uses_sea_level_as_reference(void) {
    height_init();
    fake_up = 23843;   /* any pressure */
    h_sample();
    vec3_t p;
    get_position(&p);
    int32_t pa;
    bmp180_read_pressure(&bmp, &pa);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, bmp180_pressure_to_altitude(pa, 101325.0f), p.z);
}

void test_sample_saves_the_time_in_nanoseconds(void) {
    height_init();
    mock_time_us = 2500000;   /* 2.5 s */
    h_sample();
    int64_t t;
    get_time_height(&t);
    TEST_ASSERT_EQUAL_INT64(2500000000LL, t);
}

void test_sample_does_not_change_x_and_y(void) {
    height_init();
    set_position(1.0f, 2.0f);
    h_sample();
    vec3_t p;
    get_position(&p);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, p.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, p.y);
}

void test_sample_sensor_error_keeps_the_last_value(void) {
    height_init();
    h_sample();
    vec3_t before;
    get_position(&before);
    mock_i2c_fail_from = mock_i2c_transactions + 1;   /* every next read fails */
    set_time_height(42);
    h_sample();
    vec3_t after;
    get_position(&after);
    int64_t t;
    get_time_height(&t);
    TEST_ASSERT_EQUAL_FLOAT(before.z, after.z);
    TEST_ASSERT_EQUAL_INT64_MESSAGE(42, t, "on error the timestamp must not change");
}

void test_sample_higher_pressure_lower_altitude(void) {
    height_init();
    h_sample();
    vec3_t a;
    get_position(&a);
    fake_up = 26000;   /* more pressure */
    h_sample();
    vec3_t b;
    get_position(&b);
    TEST_ASSERT_TRUE(b.z < a.z);
}
#endif

/* ------------------------------------------------------------------ */
/*                                TASK                                */
/* ------------------------------------------------------------------ */
/* Count the delays of each length done by the task. */
static int delays_of_10ms;
static void count_period(TickType_t t) { if (t == H_TASK_PERIOD_MS) delays_of_10ms++; }

void test_task_period_is_10_ms(void) {
    TEST_ASSERT_EQUAL_INT(10, H_TASK_PERIOD_MS);
    height_init();
    delays_of_10ms = 0;
    mock_delay_hook = count_period;
    mock_run_task(bmp180_task, NULL, 40);
#ifdef CONFIG_SIMULATION_ON
    TEST_ASSERT_EQUAL_INT(40, delays_of_10ms);   /* in simulation the task only waits */
#else
    /* each cycle: the conversions of the BMP180 (5 and 8 ms), then the
     * period. There is at least one period every 4 delays. */
    TEST_ASSERT_TRUE(delays_of_10ms >= 40 / 4);
#endif
}

#ifdef CONFIG_SIMULATION_ON
void test_task_simulation_does_not_touch_the_state(void) {
    height_init();
    set_h(1.23f);
    mock_run_task(bmp180_task, NULL, 10);
    vec3_t p;
    get_position(&p);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.23f, p.z);   /* in simulation ROS writes the height */
}
#else
void test_task_reads_the_sensor_every_cycle(void) {
    height_init();
    mock_i2c_log_len = 0;
    delays_of_10ms = 0;
    mock_delay_hook = count_period;
    mock_run_task(bmp180_task, NULL, 40);
    int pressure_cmds = 0;
    for (int i = 0; i < mock_i2c_log_len; i++)
        if (!mock_i2c_log[i].is_read && mock_i2c_log[i].reg == 0xF4 && (mock_i2c_log[i].value & 0x3F) == 0x34)
            pressure_cmds++;
    TEST_ASSERT_TRUE(delays_of_10ms > 0);
    TEST_ASSERT_EQUAL_INT_MESSAGE(delays_of_10ms, pressure_cmds, "one pressure reading per period");
}
#endif

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_init_test_is_false_before_init);
    RUN_TEST(test_init_test_is_true_after_init);
    RUN_TEST(test_init_creates_the_height_task);
    RUN_TEST(test_init_task_uses_the_menuconfig_stack_and_priority);
    RUN_TEST(test_init_twice_creates_one_task);
#ifdef CONFIG_SIMULATION_ON
    RUN_TEST(test_init_simulation_does_not_use_the_i2c_bus);
#else
    RUN_TEST(test_init_configures_i2c_bus_0_with_the_menuconfig_pins);
    RUN_TEST(test_init_bus_is_configured_as_master);
    RUN_TEST(test_init_detects_the_bmp180);
    RUN_TEST(test_init_no_error_check_failure_when_all_is_ok);
    RUN_TEST(test_init_works_when_the_imu_already_installed_the_bus);
    RUN_TEST(test_init_missing_sensor_does_not_delete_the_calling_task);
    RUN_TEST(test_init_missing_sensor_reports_the_failure);
#endif

    RUN_TEST(test_data_null_is_invalid_arg);
    RUN_TEST(test_data_before_init_is_invalid_state);
    RUN_TEST(test_data_after_init_is_ok);
    RUN_TEST(test_data_uses_the_critical_section);
#ifndef CONFIG_SIMULATION_ON
    RUN_TEST(test_data_returns_the_last_altitude);

    RUN_TEST(test_sample_saves_the_altitude_in_the_state);
    RUN_TEST(test_sample_uses_sea_level_as_reference);
    RUN_TEST(test_sample_saves_the_time_in_nanoseconds);
    RUN_TEST(test_sample_does_not_change_x_and_y);
    RUN_TEST(test_sample_sensor_error_keeps_the_last_value);
    RUN_TEST(test_sample_higher_pressure_lower_altitude);
#endif

    RUN_TEST(test_task_period_is_10_ms);
#ifdef CONFIG_SIMULATION_ON
    RUN_TEST(test_task_simulation_does_not_touch_the_state);
#else
    RUN_TEST(test_task_reads_the_sensor_every_cycle);
#endif

    return UNITY_END();
}
