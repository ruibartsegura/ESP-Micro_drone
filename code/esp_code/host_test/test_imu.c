/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for imu.c (host, no hardware). The file is built twice:
 *     - imu_hw:  real drone. A fake MPU-6050 is simulated on the mocked
 *                I2C bus; the real mpu6050.c and state.c are linked.
 *     - imu_sim: CONFIG_SIMULATION_ON. The sensor is not used (the IMU
 *                data comes from ROS), only the task is started.
 *   They check the init (bus, sensor, calibration, task), the reading of
 *   the samples into the global state and the stability check done before
 *   arming the drone.
 *
 *   Units of the global state written by this module: acceleration in g
 *   (Z = +1 g at rest), angular velocity in deg/s, time in ns.
 *
 *   imu_check_stable_for_arming() reads the global state: the tests write
 *   one fake sample in the state for each vTaskDelay() of the check.
 *
 *   The .c file is included directly. Another version can be tested with:
 *       make run T=imu_hw MODULE_SRC=path/to/imu.c
 *
 *   Tests that fail with the current code have [BUG] in their comment.
 *
 * Functions:
 *   - fake_load(): writes the current fake sample in the global state.
 *   - fake_mpu_set_sample(): (hardware) raw sample in the fake sensor.
 *   - setUp() / tearDown(): reset the mocks, the module and the state.
 *   - test_init_*: imu_init() and imu_test().
 *   - test_sample_*: (hardware) imu_sample_and_filter().
 *   - test_task_*: the FreeRTOS task.
 *   - test_arm_*: imu_check_stable_for_arming().
 *   - main(): runs all the tests.
 */
#include "unity.h"
#include "test_helpers.h"
#include "mocks/mock_reset.h"

#ifndef MODULE_SRC
#define MODULE_SRC "../components/drivers/imu/imu.c"
#endif
#include MODULE_SRC

#define MPU_ADDR MPU6050_ADDR

/* ------------------------------------------------------------------ */
/*                           FAKE SAMPLES                             */
/* ------------------------------------------------------------------ */
#define MAX_SAMPLES 64
static IMU fake_samples[MAX_SAMPLES];
static int fake_n;          /* samples available (the last one repeats) */
static int fake_k;          /* current sample */

/* The current sample is also the global state: like the real IMU task. */
static void fake_load(void) {
    const IMU *s = &fake_samples[fake_k];
    set_acc_lin(s->Acc_lin_X, s->Acc_lin_Y, s->Acc_lin_Z);
    set_vel_ang(s->Vel_ang_X, s->Vel_ang_Y, s->Vel_ang_Z);
    set_time_imu(s->Time_stamp);
}

/* Each vTaskDelay() of the check moves to the next sample. */
static void fake_next_sample(TickType_t t) {
    (void)t;
    if (fake_k < fake_n - 1) fake_k++;
    fake_load();
}

static IMU sample(float ax, float ay, float az, float gx, float gy, float gz) {
    IMU s = { 0 };
    s.Acc_lin_X = ax; s.Acc_lin_Y = ay; s.Acc_lin_Z = az;
    s.Vel_ang_X = gx; s.Vel_ang_Y = gy; s.Vel_ang_Z = gz;
    return s;
}

static void fake_all(IMU s) {
    for (int i = 0; i < MAX_SAMPLES; i++) fake_samples[i] = s;
    fake_n = MAX_SAMPLES;
}

/* Prepare the stability check: the drone is initialised and the first
 * sample is in the state. */
static void arm_check_setup(void) {
    is_init = true;
    fake_k = 0;
    fake_load();
    mock_delay_hook = fake_next_sample;
}

#ifndef CONFIG_SIMULATION_ON
static void fake_mpu_set_sample(int16_t ax, int16_t ay, int16_t az,
                                int16_t gx, int16_t gy, int16_t gz) {
    mock_i2c_set_be16(MPU_ADDR, 0x3B, ax);
    mock_i2c_set_be16(MPU_ADDR, 0x3D, ay);
    mock_i2c_set_be16(MPU_ADDR, 0x3F, az);
    mock_i2c_set_be16(MPU_ADDR, 0x43, gx);
    mock_i2c_set_be16(MPU_ADDR, 0x45, gy);
    mock_i2c_set_be16(MPU_ADDR, 0x47, gz);
}

static void init_guarded(void) {
    TEST_ASSERT_NO_CRASH(imu_init(), "[BUG] imu_init() crashes in the calibration (NULL timestamp in mpu6050.c)");
}
#endif

static void reset_module(void) {
    is_init = false;
#ifndef CONFIG_SIMULATION_ON
    for (int i = 0; i < 3; i++) accel_bias[i] = gyro_bias[i] = 0.0f;
#endif
}

void setUp(void) {
    mock_reset_all();
    reset_module();
    state_init();
    memset(fake_samples, 0, sizeof fake_samples);
    fake_all(sample(0, 0, 1.0f, 0, 0, 0));
    fake_k = 0;
#ifndef CONFIG_SIMULATION_ON
    fake_mpu_set_sample(0, 0, 16384, 0, 0, 0);   /* level and still */
#endif
}

void tearDown(void) {
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, mock_critical_depth, "a critical section was not released");
}

/* ------------------------------------------------------------------ */
/*                               INIT                                 */
/* ------------------------------------------------------------------ */
void test_init_test_is_false_before_init(void) {
    TEST_ASSERT_FALSE(imu_test());
}

#ifdef CONFIG_SIMULATION_ON
void test_init_test_is_true_after_init(void) {
    imu_init();
    TEST_ASSERT_TRUE(imu_test());
}

void test_init_creates_the_task_with_the_menuconfig_values(void) {
    imu_init();
    const mock_task_t *t = mock_find_task("imu_task");
    TEST_ASSERT_NOT_NULL(t);
    TEST_ASSERT_EQUAL_UINT32(CONFIG_IMU_TASK_STACK, t->stack);
    TEST_ASSERT_EQUAL_UINT(CONFIG_IMU_TASK_PRIO, t->prio);
}

void test_init_twice_creates_one_task(void) {
    imu_init();
    imu_init();
    TEST_ASSERT_EQUAL_INT(1, mock_task_count);
}

void test_init_simulation_does_not_use_the_i2c_bus(void) {
    imu_init();
    TEST_ASSERT_EQUAL_INT(0, mock_i2c_param_config_calls);
    TEST_ASSERT_EQUAL_INT(0, mock_i2c_transactions);
}
#else
void test_init_test_is_true_after_init(void) {
    init_guarded();
    TEST_ASSERT_TRUE(imu_test());
}

void test_init_creates_the_task_with_the_menuconfig_values(void) {
    init_guarded();
    const mock_task_t *t = mock_find_task("imu_task");
    TEST_ASSERT_NOT_NULL(t);
    TEST_ASSERT_EQUAL_UINT32(CONFIG_IMU_TASK_STACK, t->stack);
    TEST_ASSERT_EQUAL_UINT(CONFIG_IMU_TASK_PRIO, t->prio);
}

void test_init_twice_creates_one_task(void) {
    init_guarded();
    init_guarded();
    TEST_ASSERT_EQUAL_INT(1, mock_task_count);
}

void test_init_configures_i2c_bus_0_as_master(void) {
    init_guarded();
    TEST_ASSERT_EQUAL_INT(MOCK_I2C_MODE_MASTER, mock_i2c_conf[I2C_NUM_0].mode);
    TEST_ASSERT_EQUAL_INT(CONFIG_I2C0_PIN_SDA, mock_i2c_conf[I2C_NUM_0].sda_io_num);
    TEST_ASSERT_EQUAL_INT(CONFIG_I2C0_PIN_SCL, mock_i2c_conf[I2C_NUM_0].scl_io_num);
    TEST_ASSERT_EQUAL_INT(GPIO_PULLUP_ENABLE, mock_i2c_conf[I2C_NUM_0].sda_pullup_en);
    TEST_ASSERT_EQUAL_INT(GPIO_PULLUP_ENABLE, mock_i2c_conf[I2C_NUM_0].scl_pullup_en);
    TEST_ASSERT_EQUAL_UINT32(400000, mock_i2c_conf[I2C_NUM_0].master.clk_speed);
    TEST_ASSERT_EQUAL_INT(MOCK_I2C_MODE_MASTER, mock_i2c_installed_mode[I2C_NUM_0]);
}

void test_init_wakes_up_the_mpu6050(void) {
    init_guarded();
    TEST_ASSERT_EQUAL_INT(1, mock_i2c_count_writes(MPU_ADDR, MPU6050_REG_PWR_MGMT_1));
}

void test_init_calibrates_the_sensor(void) {
    fake_mpu_set_sample(164, 0, 16384, 262, 0, 0);   /* 0.01 g and 2 deg/s of offset */
    init_guarded();
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.01f, accel_bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f, gyro_bias[0]);
}

void test_init_bus_config_error_stops_the_init(void) {
    mock_i2c_param_config_ret = ESP_FAIL;
    imu_init();
    TEST_ASSERT_FALSE(imu_test());
    TEST_ASSERT_EQUAL_INT(0, mock_task_count);
}

void test_init_driver_install_error_stops_the_init(void) {
    mock_i2c_driver_install_ret = ESP_FAIL;
    imu_init();
    TEST_ASSERT_FALSE(imu_test());
    TEST_ASSERT_EQUAL_INT(0, mock_task_count);
}

void test_init_sensor_error_stops_the_init(void) {
    mock_i2c_fail_from = 1;   /* the MPU-6050 does not answer */
    imu_init();
    TEST_ASSERT_FALSE(imu_test());
    TEST_ASSERT_EQUAL_INT(0, mock_task_count);
}

void test_init_calibration_error_stops_the_init(void) {
    mock_i2c_fail_at = 5 + 50;   /* the init uses 5 writes, then fails in the calibration */
    TEST_ASSERT_NO_CRASH(imu_init(), "[BUG] imu_init() crashes in the calibration (NULL timestamp in mpu6050.c)");
    TEST_ASSERT_FALSE(imu_test());
    TEST_ASSERT_EQUAL_INT(0, mock_task_count);
}

/* ------------------------------------------------------------------ */
/*                     SAMPLES (imu_sample_and_filter)                */
/* ------------------------------------------------------------------ */
void test_sample_saves_acceleration_in_g(void) {
    fake_mpu_set_sample(8192, -16384, 16384, 0, 0, 0);
    imu_sample_and_filter();
    vec3_t a;
    get_acc_lin(&a);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, a.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, a.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, a.z);
}

void test_sample_saves_angular_velocity_in_deg_per_s(void) {
    fake_mpu_set_sample(0, 0, 16384, 131, -262, 1310);
    imu_sample_and_filter();
    vec3_t w;
    get_vel_ang(&w);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, w.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -2.0f, w.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f, w.z);
}

void test_sample_removes_the_calibration_bias(void) {
    accel_bias[0] = 0.5f;
    gyro_bias[2] = 10.0f;
    fake_mpu_set_sample(8192, 0, 16384, 0, 0, 1310);
    imu_sample_and_filter();
    vec3_t a, w;
    get_acc_lin(&a);
    get_vel_ang(&w);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, a.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, w.z);
}

void test_sample_saves_the_time_in_nanoseconds(void) {
    mock_time_us = 3000000;
    imu_sample_and_filter();
    int64_t t;
    get_time_imu(&t);
    TEST_ASSERT_EQUAL_INT64(3000000000LL, t);
}

void test_sample_sensor_error_keeps_the_last_value(void) {
    fake_mpu_set_sample(0, 0, 16384, 131, 0, 0);
    mock_time_us = 1000;
    imu_sample_and_filter();
    mock_i2c_fail_from = mock_i2c_transactions + 1;
    fake_mpu_set_sample(0, 0, 0, 0, 0, 0);
    mock_time_us = 2000;
    imu_sample_and_filter();
    vec3_t a, w;
    int64_t t;
    get_acc_lin(&a);
    get_vel_ang(&w);
    get_time_imu(&t);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, a.z);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, w.x);
    TEST_ASSERT_EQUAL_INT64(1000000LL, t);
}

void test_sample_does_not_change_the_position(void) {
    set_position(1.0f, 2.0f);
    set_h(3.0f);
    imu_sample_and_filter();
    vec3_t p;
    get_position(&p);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.0f, p.z);
}
#endif

/* ------------------------------------------------------------------ */
/*                                TASK                                */
/* ------------------------------------------------------------------ */
static int delays_of_period;
static void count_period(TickType_t t) { if (t == IMU_TASK_PERIOD_MS) delays_of_period++; }

void test_task_period_is_10_ms_100_hz(void) {
    TEST_ASSERT_EQUAL_INT(10, IMU_TASK_PERIOD_MS);
    delays_of_period = 0;
    mock_delay_hook = count_period;
    mock_run_task(imu_task, NULL, 20);
    TEST_ASSERT_EQUAL_INT(20, delays_of_period);
}

#ifdef CONFIG_SIMULATION_ON
void test_task_simulation_does_not_touch_the_state(void) {
    set_acc_lin(0.1f, 0.2f, 9.8f);
    mock_run_task(imu_task, NULL, 10);
    vec3_t a;
    get_acc_lin(&a);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 9.8f, a.z);   /* in simulation ROS writes it */
    TEST_ASSERT_EQUAL_INT(0, mock_i2c_transactions);
}
#else
void test_task_reads_the_sensor_every_cycle(void) {
    mock_run_task(imu_task, NULL, 20);
    TEST_ASSERT_EQUAL_INT(20, mock_i2c_transactions);
}
#endif

/* ------------------------------------------------------------------ */
/*                       STABILITY BEFORE ARMING                      */
/* ------------------------------------------------------------------ */
void test_arm_null_is_invalid_arg(void) {
    arm_check_setup();
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, imu_check_stable_for_arming(NULL));
}

void test_arm_not_initialised_is_invalid_state(void) {
    imu_arm_state_t st = IMU_ARM_STATE_MOVING;
    arm_check_setup();
    is_init = false;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, imu_check_stable_for_arming(&st));
    TEST_ASSERT_EQUAL_INT(IMU_ARM_STATE_MOVING, st);
}

void test_arm_level_and_still_is_stable(void) {
    imu_arm_state_t st = IMU_ARM_STATE_MOVING;
    arm_check_setup();
    TEST_ASSERT_EQUAL_INT(ESP_OK, imu_check_stable_for_arming(&st));
    TEST_ASSERT_EQUAL_INT(IMU_ARM_STATE_STABLE, st);
}

void test_arm_checks_20_samples_10_ms_apart(void) {
    imu_arm_state_t st;
    arm_check_setup();
    imu_check_stable_for_arming(&st);
    TEST_ASSERT_EQUAL_INT(IMU_ARM_CHECK_SAMPLES, mock_delay_calls);   /* one wait per sample */
    TEST_ASSERT_EQUAL_UINT64(IMU_ARM_CHECK_SAMPLES * IMU_ARM_CHECK_PERIOD_MS, mock_delay_total_ms);
}

void test_arm_rotating_drone_is_moving(void) {
    imu_arm_state_t st;
    fake_all(sample(0, 0, 1.0f, 0, 0, 20.0f));
    arm_check_setup();
    imu_check_stable_for_arming(&st);
    TEST_ASSERT_EQUAL_INT(IMU_ARM_STATE_MOVING, st);
}

void test_arm_gyro_limit_uses_the_magnitude(void) {
    /* 4 deg/s on each axis: every axis < 5 but |w| = 6.9 > 5 */
    imu_arm_state_t st;
    fake_all(sample(0, 0, 1.0f, 4.0f, 4.0f, 4.0f));
    arm_check_setup();
    imu_check_stable_for_arming(&st);
    TEST_ASSERT_EQUAL_INT(IMU_ARM_STATE_MOVING, st);
}

void test_arm_small_gyro_noise_is_stable(void) {
    imu_arm_state_t st;
    fake_all(sample(0.01f, -0.01f, 1.0f, 1.0f, -1.0f, 0.5f));
    arm_check_setup();
    imu_check_stable_for_arming(&st);
    TEST_ASSERT_EQUAL_INT(IMU_ARM_STATE_STABLE, st);
}

void test_arm_one_bad_sample_is_enough_and_stops_early(void) {
    imu_arm_state_t st;
    fake_all(sample(0, 0, 1.0f, 0, 0, 0));
    fake_samples[5] = sample(0, 0, 1.0f, 50.0f, 0, 0);
    arm_check_setup();
    imu_check_stable_for_arming(&st);
    TEST_ASSERT_EQUAL_INT(IMU_ARM_STATE_MOVING, st);
    TEST_ASSERT_EQUAL_INT_MESSAGE(5, mock_delay_calls, "it must stop at the 6th sample");
}

void test_arm_low_acceleration_is_abnormal(void) {
    imu_arm_state_t st;
    fake_all(sample(0, 0, 0.5f, 0, 0, 0));   /* falling */
    arm_check_setup();
    imu_check_stable_for_arming(&st);
    TEST_ASSERT_EQUAL_INT(IMU_ARM_STATE_ACCEL_ABNORMAL, st);
}

void test_arm_high_acceleration_is_abnormal(void) {
    imu_arm_state_t st;
    fake_all(sample(0, 0, 1.3f, 0, 0, 0));   /* hand shaking the drone */
    arm_check_setup();
    imu_check_stable_for_arming(&st);
    TEST_ASSERT_EQUAL_INT(IMU_ARM_STATE_ACCEL_ABNORMAL, st);
}

void test_arm_acceleration_limits_are_inclusive(void) {
    imu_arm_state_t st;
    fake_all(sample(0, 0, IMU_ARM_ACCEL_MIN_G + 0.001f, 0, 0, 0));
    arm_check_setup();
    imu_check_stable_for_arming(&st);
    TEST_ASSERT_EQUAL_INT(IMU_ARM_STATE_STABLE, st);
    fake_all(sample(0, 0, IMU_ARM_ACCEL_MAX_G - 0.001f, 0, 0, 0));
    arm_check_setup();
    imu_check_stable_for_arming(&st);
    TEST_ASSERT_EQUAL_INT(IMU_ARM_STATE_STABLE, st);
}

/* [BUG] imu.h defines IMU_ARM_MAX_TILT_DEG (10 deg) and the state
 * IMU_ARM_STATE_TILTED, but the tilt is never checked: a drone on a slope
 * of 30 deg is "stable" and can be armed. */
void test_arm_tilted_drone_is_tilted(void) {
    imu_arm_state_t st;
    fake_all(sample(0, sinf(RAD(30.0f)), cosf(RAD(30.0f)), 0, 0, 0));   /* |a| = 1 g, roll 30 deg */
    arm_check_setup();
    imu_check_stable_for_arming(&st);
    TEST_ASSERT_EQUAL_INT(IMU_ARM_STATE_TILTED, st);
}

void test_arm_small_tilt_is_stable(void) {
    imu_arm_state_t st;
    fake_all(sample(-sinf(RAD(3.0f)), 0, cosf(RAD(3.0f)), 0, 0, 0));
    arm_check_setup();
    imu_check_stable_for_arming(&st);
    TEST_ASSERT_EQUAL_INT(IMU_ARM_STATE_STABLE, st);
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_init_test_is_false_before_init);
    RUN_TEST(test_init_test_is_true_after_init);
    RUN_TEST(test_init_creates_the_task_with_the_menuconfig_values);
    RUN_TEST(test_init_twice_creates_one_task);
#ifdef CONFIG_SIMULATION_ON
    RUN_TEST(test_init_simulation_does_not_use_the_i2c_bus);
#else
    RUN_TEST(test_init_configures_i2c_bus_0_as_master);
    RUN_TEST(test_init_wakes_up_the_mpu6050);
    RUN_TEST(test_init_calibrates_the_sensor);
    RUN_TEST(test_init_bus_config_error_stops_the_init);
    RUN_TEST(test_init_driver_install_error_stops_the_init);
    RUN_TEST(test_init_sensor_error_stops_the_init);
    RUN_TEST(test_init_calibration_error_stops_the_init);

    RUN_TEST(test_sample_saves_acceleration_in_g);
    RUN_TEST(test_sample_saves_angular_velocity_in_deg_per_s);
    RUN_TEST(test_sample_removes_the_calibration_bias);
    RUN_TEST(test_sample_saves_the_time_in_nanoseconds);
    RUN_TEST(test_sample_sensor_error_keeps_the_last_value);
    RUN_TEST(test_sample_does_not_change_the_position);
#endif

    RUN_TEST(test_task_period_is_10_ms_100_hz);
#ifdef CONFIG_SIMULATION_ON
    RUN_TEST(test_task_simulation_does_not_touch_the_state);
#else
    RUN_TEST(test_task_reads_the_sensor_every_cycle);
#endif

    RUN_TEST(test_arm_null_is_invalid_arg);
    RUN_TEST(test_arm_not_initialised_is_invalid_state);
    RUN_TEST(test_arm_level_and_still_is_stable);
    RUN_TEST(test_arm_checks_20_samples_10_ms_apart);
    RUN_TEST(test_arm_rotating_drone_is_moving);
    RUN_TEST(test_arm_gyro_limit_uses_the_magnitude);
    RUN_TEST(test_arm_small_gyro_noise_is_stable);
    RUN_TEST(test_arm_one_bad_sample_is_enough_and_stops_early);
    RUN_TEST(test_arm_low_acceleration_is_abnormal);
    RUN_TEST(test_arm_high_acceleration_is_abnormal);
    RUN_TEST(test_arm_acceleration_limits_are_inclusive);
    RUN_TEST(test_arm_tilted_drone_is_tilted);
    RUN_TEST(test_arm_small_tilt_is_stable);

    return UNITY_END();
}
