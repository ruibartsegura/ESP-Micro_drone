/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for mpu6050.c (host, no hardware). A fake MPU-6050 is
 *   simulated on the mocked I2C bus (address 0x68), so the tests check the
 *   registers written by the init, the parsing of the raw data, the unit
 *   conversion and the calibration.
 *
 *   Units: accelerometer in g (+-2 g, 16384 LSB/g), gyroscope in deg/s
 *   (+-250 deg/s, 131 LSB/(deg/s)), timestamp in nanoseconds.
 *
 *   The .c file is included directly. Another version can be tested with:
 *       make run T=mpu6050 MODULE_SRC=path/to/mpu6050.c
 *
 *   Tests that fail with the current code have [BUG] in their comment.
 *
 * Functions:
 *   - fake_mpu_set_sample(): puts a raw sample in the fake sensor.
 *   - setUp() / tearDown(): reset the mocks before each test.
 *   - test_init_*: mpu6050_init().
 *   - test_raw_*: mpu6050_read_raw_data().
 *   - test_conv_*: mpu6050_convert_accel() / mpu6050_convert_gyro().
 *   - test_calib_*: mpu6050_calibrate().
 *   - main(): runs all the tests.
 */
#include "unity.h"
#include "test_helpers.h"
#include "mocks/mock_reset.h"

#ifndef MODULE_SRC
#define MODULE_SRC "../components/drivers/imu/mpu6050.c"
#endif
#include MODULE_SRC

#define ADDR MPU6050_ADDR
#define EPS  1e-4f

static const float ZERO_BIAS[3] = { 0.0f, 0.0f, 0.0f };

/* Raw sample in the data registers 0x3B..0x48 (temperature in the middle). */
static void fake_mpu_set_sample(int16_t ax, int16_t ay, int16_t az,
                                int16_t gx, int16_t gy, int16_t gz) {
    mock_i2c_set_be16(ADDR, 0x3B, ax);
    mock_i2c_set_be16(ADDR, 0x3D, ay);
    mock_i2c_set_be16(ADDR, 0x3F, az);
    mock_i2c_set_be16(ADDR, 0x41, 0x1234);   /* temperature, must be ignored */
    mock_i2c_set_be16(ADDR, 0x43, gx);
    mock_i2c_set_be16(ADDR, 0x45, gy);
    mock_i2c_set_be16(ADDR, 0x47, gz);
}

/* Position of the first write of a register in the I2C log (-1 if none). */
static int first_write(uint8_t reg) {
    for (int i = 0; i < mock_i2c_log_len; i++)
        if (!mock_i2c_log[i].is_read && mock_i2c_log[i].reg == reg) return i;
    return -1;
}

void setUp(void) {
    mock_reset_all();
    for (int r = 0; r < 256; r++) mock_i2c_regs[ADDR][r] = 0xFF;   /* reset value != 0 */
    fake_mpu_set_sample(0, 0, 16384, 0, 0, 0);                    /* level and still */
}

void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                               INIT                                 */
/* ------------------------------------------------------------------ */
void test_init_returns_ok(void) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, mpu6050_init(I2C_NUM_0));
}

void test_init_wakes_up_the_sensor(void) {
    mpu6050_init(I2C_NUM_0);
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(0x00, mock_i2c_regs[ADDR][MPU6050_REG_PWR_MGMT_1],
                                   "PWR_MGMT_1 = 0 clears the SLEEP bit");
}

void test_init_wake_up_is_the_first_write(void) {
    mpu6050_init(I2C_NUM_0);
    TEST_ASSERT_EQUAL_INT(0, first_write(MPU6050_REG_PWR_MGMT_1));
}

void test_init_waits_after_wake_up(void) {
    mpu6050_init(I2C_NUM_0);
    TEST_ASSERT_TRUE(mock_delay_total_ms >= 50);
}

void test_init_sets_gyro_range_250_dps(void) {
    mpu6050_init(I2C_NUM_0);
    TEST_ASSERT_EQUAL_HEX8(0x00, mock_i2c_regs[ADDR][MPU6050_REG_GYRO_CONFIG]);
}

void test_init_sets_accel_range_2g(void) {
    mpu6050_init(I2C_NUM_0);
    TEST_ASSERT_EQUAL_HEX8(0x00, mock_i2c_regs[ADDR][MPU6050_REG_ACCEL_CONFIG]);
}

void test_init_sets_low_pass_filter_44hz(void) {
    mpu6050_init(I2C_NUM_0);
    TEST_ASSERT_EQUAL_HEX8(0x03, mock_i2c_regs[ADDR][MPU6050_REG_CONFIG]);
}

void test_init_sets_sample_rate_divider_0(void) {
    mpu6050_init(I2C_NUM_0);
    TEST_ASSERT_EQUAL_HEX8(0x00, mock_i2c_regs[ADDR][MPU6050_REG_SMPLRT_DIV]);
}

void test_init_writes_every_register_once(void) {
    mpu6050_init(I2C_NUM_0);
    TEST_ASSERT_EQUAL_INT(1, mock_i2c_count_writes(ADDR, MPU6050_REG_PWR_MGMT_1));
    TEST_ASSERT_EQUAL_INT(1, mock_i2c_count_writes(ADDR, MPU6050_REG_GYRO_CONFIG));
    TEST_ASSERT_EQUAL_INT(1, mock_i2c_count_writes(ADDR, MPU6050_REG_ACCEL_CONFIG));
    TEST_ASSERT_EQUAL_INT(1, mock_i2c_count_writes(ADDR, MPU6050_REG_CONFIG));
    TEST_ASSERT_EQUAL_INT(1, mock_i2c_count_writes(ADDR, MPU6050_REG_SMPLRT_DIV));
}

void test_init_error_in_any_write_is_returned_and_stops(void) {
    for (int step = 1; step <= 5; step++) {
        mock_reset_all();
        mock_i2c_fail_at = step;
        mock_i2c_fail_err = ESP_ERR_TIMEOUT;
        TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT, mpu6050_init(I2C_NUM_0));
        TEST_ASSERT_EQUAL_INT_MESSAGE(step, mock_i2c_transactions, "the init must stop at the first error");
    }
}

/* ------------------------------------------------------------------ */
/*                             RAW DATA                               */
/* ------------------------------------------------------------------ */
void test_raw_reads_14_bytes_from_0x3B_in_one_transaction(void) {
    int16_t ax, ay, az, gx, gy, gz;
    int64_t t;
    mpu6050_read_raw_data(I2C_NUM_0, &ax, &ay, &az, &gx, &gy, &gz, &t);
    TEST_ASSERT_EQUAL_INT(1, mock_i2c_transactions);
    TEST_ASSERT_TRUE(mock_i2c_log[0].is_read);
    TEST_ASSERT_EQUAL_HEX8(MPU6050_REG_ACCEL_XOUT_H, mock_i2c_log[0].reg);
    TEST_ASSERT_EQUAL_UINT(14, mock_i2c_log[0].len);
}

void test_raw_parses_big_endian_values(void) {
    int16_t ax, ay, az, gx, gy, gz;
    int64_t t;
    fake_mpu_set_sample(1, 256, 0x1234, -1, -256, 32767);
    TEST_ASSERT_EQUAL_INT(ESP_OK, mpu6050_read_raw_data(I2C_NUM_0, &ax, &ay, &az, &gx, &gy, &gz, &t));
    TEST_ASSERT_EQUAL_INT16(1, ax);
    TEST_ASSERT_EQUAL_INT16(256, ay);
    TEST_ASSERT_EQUAL_INT16(0x1234, az);
    TEST_ASSERT_EQUAL_INT16(-1, gx);
    TEST_ASSERT_EQUAL_INT16(-256, gy);
    TEST_ASSERT_EQUAL_INT16(32767, gz);
}

void test_raw_negative_full_scale(void) {
    int16_t ax, ay, az, gx, gy, gz;
    int64_t t;
    fake_mpu_set_sample(-32768, -32768, -32768, -32768, -32768, -32768);
    mpu6050_read_raw_data(I2C_NUM_0, &ax, &ay, &az, &gx, &gy, &gz, &t);
    TEST_ASSERT_EQUAL_INT16(-32768, ax);
    TEST_ASSERT_EQUAL_INT16(-32768, gz);
}

void test_raw_ignores_the_temperature_registers(void) {
    int16_t ax, ay, az, gx, gy, gz;
    int64_t t;
    fake_mpu_set_sample(10, 20, 30, 40, 50, 60);
    mpu6050_read_raw_data(I2C_NUM_0, &ax, &ay, &az, &gx, &gy, &gz, &t);
    TEST_ASSERT_EQUAL_INT16(40, gx);   /* gyro starts at 0x43, after the temperature */
}

void test_raw_timestamp_is_in_nanoseconds(void) {
    int16_t ax, ay, az, gx, gy, gz;
    int64_t t = 0;
    mock_time_us = 1234567;
    mpu6050_read_raw_data(I2C_NUM_0, &ax, &ay, &az, &gx, &gy, &gz, &t);
    TEST_ASSERT_EQUAL_INT64(1234567000LL, t);
}

void test_raw_i2c_error_is_returned_and_outputs_not_changed(void) {
    int16_t ax = 7, ay = 7, az = 7, gx = 7, gy = 7, gz = 7;
    int64_t t = 7;
    mock_i2c_fail_at = 1;
    mock_i2c_fail_err = ESP_ERR_TIMEOUT;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT, mpu6050_read_raw_data(I2C_NUM_0, &ax, &ay, &az, &gx, &gy, &gz, &t));
    TEST_ASSERT_EQUAL_INT16(7, ax);
    TEST_ASSERT_EQUAL_INT16(7, gz);
    TEST_ASSERT_EQUAL_INT64(7, t);
}

/* [BUG] mpu6050_calibrate() calls this function with t_stamp = NULL, and
 * the function always writes *t_stamp: NULL pointer access. The timestamp
 * must be optional. */
void test_raw_null_timestamp_is_allowed(void) {
    int16_t ax, ay, az, gx, gy, gz;
    esp_err_t ret = ESP_FAIL;
    TEST_ASSERT_NO_CRASH(ret = mpu6050_read_raw_data(I2C_NUM_0, &ax, &ay, &az, &gx, &gy, &gz, NULL),
                         "crash writing the timestamp in a NULL pointer");
    TEST_ASSERT_EQUAL_INT(ESP_OK, ret);
}

/* ------------------------------------------------------------------ */
/*                            CONVERSION                              */
/* ------------------------------------------------------------------ */
void test_conv_accel_16384_is_1g(void) {
    float x, y, z;
    mpu6050_convert_accel(16384, -16384, 8192, ZERO_BIAS, &x, &y, &z);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -1.0f, y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.5f, z);
}

void test_conv_accel_full_scale_is_2g(void) {
    float x, y, z;
    mpu6050_convert_accel(-32768, 32767, 0, ZERO_BIAS, &x, &y, &z);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -2.0f, x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f, y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, z);
}

void test_conv_accel_removes_the_bias(void) {
    const float bias[3] = { 0.1f, -0.2f, 0.05f };
    float x, y, z;
    mpu6050_convert_accel(16384, 0, 16384, bias, &x, &y, &z);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.9f, x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.2f, y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.95f, z);
}

void test_conv_gyro_131_is_1_dps(void) {
    float x, y, z;
    mpu6050_convert_gyro(131, -131, 1310, ZERO_BIAS, &x, &y, &z);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -1.0f, y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 10.0f, z);
}

void test_conv_gyro_full_scale_is_250_dps(void) {
    float x, y, z;
    mpu6050_convert_gyro(32767, -32768, 0, ZERO_BIAS, &x, &y, &z);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 250.1f, x);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, -250.1f, y);
}

void test_conv_gyro_removes_the_bias(void) {
    const float bias[3] = { 1.0f, -2.0f, 0.5f };
    float x, y, z;
    mpu6050_convert_gyro(131, 0, 0, bias, &x, &y, &z);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -0.5f, z);
}

void test_conv_does_not_change_the_bias(void) {
    float bias[3] = { 0.1f, 0.2f, 0.3f };
    float x, y, z;
    mpu6050_convert_accel(100, 100, 100, bias, &x, &y, &z);
    mpu6050_convert_gyro(100, 100, 100, bias, &x, &y, &z);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, bias[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.3f, bias[2]);
}

/* ------------------------------------------------------------------ */
/*                            CALIBRATION                             */
/* ------------------------------------------------------------------ */
/* Every calibration test is protected: with the [BUG] of the NULL
 * timestamp (see test_raw_null_timestamp_is_allowed) the calibration crashes. */
static esp_err_t calibrate(float accel_bias[3], float gyro_bias[3]) {
    esp_err_t ret = ESP_FAIL;
    TEST_ASSERT_NO_CRASH(ret = mpu6050_calibrate(I2C_NUM_0, accel_bias, gyro_bias),
                         "[BUG] mpu6050_calibrate() crashes (NULL timestamp)");
    return ret;
}

void test_calib_level_and_still_gives_zero_bias(void) {
    float ab[3] = { 9, 9, 9 }, gb[3] = { 9, 9, 9 };
    fake_mpu_set_sample(0, 0, 16384, 0, 0, 0);
    TEST_ASSERT_EQUAL_INT(ESP_OK, calibrate(ab, gb));
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, ab[i]);
        TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, gb[i]);
    }
}

void test_calib_z_bias_keeps_the_gravity(void) {
    /* At rest Z must read +1 g: the bias is the difference with 1 g */
    float ab[3], gb[3];
    fake_mpu_set_sample(0, 0, 16384 + 1638, 0, 0, 0);   /* 1.1 g */
    calibrate(ab, gb);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.1f, ab[2]);
}

void test_calib_offsets_of_every_axis(void) {
    float ab[3], gb[3];
    fake_mpu_set_sample(820, -1638, 16384, 262, -131, 655);   /* 0.05 g, -0.1 g, 2, -1, 5 deg/s */
    calibrate(ab, gb);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.05f, ab[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -0.1f, ab[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f, gb[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -1.0f, gb[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 5.0f, gb[2]);
}

void test_calib_after_calibration_the_sample_reads_0_0_1g(void) {
    float ab[3], gb[3], x, y, z, wx, wy, wz;
    fake_mpu_set_sample(500, -300, 17000, 100, -50, 20);
    calibrate(ab, gb);
    mpu6050_convert_accel(500, -300, 17000, ab, &x, &y, &z);
    mpu6050_convert_gyro(100, -50, 20, gb, &wx, &wy, &wz);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, y);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, z);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, wx);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, wy);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, wz);
}

/* Fake noise: every read gives a different sample, the bias is the mean. */
static int noise_i;
static void noisy_read_hook(uint8_t addr, uint8_t reg, size_t len) {
    if (addr != ADDR || reg != 0x3B) return;
    int16_t n = (noise_i++ % 2) ? 131 : -131;   /* +-1 deg/s, mean 0 */
    fake_mpu_set_sample(0, 0, 16384, (int16_t)(262 + n), 0, 0);
}

void test_calib_bias_is_the_mean_of_the_samples(void) {
    float ab[3], gb[3];
    noise_i = 0;
    mock_i2c_read_hook = noisy_read_hook;
    calibrate(ab, gb);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f, gb[0]);
}

void test_calib_uses_200_samples(void) {
    float ab[3], gb[3];
    calibrate(ab, gb);
    TEST_ASSERT_EQUAL_INT(200, mock_i2c_transactions);
    TEST_ASSERT_EQUAL_INT(200, mock_delay_calls);
}

void test_calib_i2c_error_is_returned_and_bias_not_changed(void) {
    float ab[3] = { 7, 7, 7 }, gb[3] = { 7, 7, 7 };
    mock_i2c_fail_at = 10;
    mock_i2c_fail_err = ESP_ERR_TIMEOUT;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT, calibrate(ab, gb));
    TEST_ASSERT_EQUAL_FLOAT(7.0f, ab[0]);
    TEST_ASSERT_EQUAL_FLOAT(7.0f, gb[2]);
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_init_returns_ok);
    RUN_TEST(test_init_wakes_up_the_sensor);
    RUN_TEST(test_init_wake_up_is_the_first_write);
    RUN_TEST(test_init_waits_after_wake_up);
    RUN_TEST(test_init_sets_gyro_range_250_dps);
    RUN_TEST(test_init_sets_accel_range_2g);
    RUN_TEST(test_init_sets_low_pass_filter_44hz);
    RUN_TEST(test_init_sets_sample_rate_divider_0);
    RUN_TEST(test_init_writes_every_register_once);
    RUN_TEST(test_init_error_in_any_write_is_returned_and_stops);

    RUN_TEST(test_raw_reads_14_bytes_from_0x3B_in_one_transaction);
    RUN_TEST(test_raw_parses_big_endian_values);
    RUN_TEST(test_raw_negative_full_scale);
    RUN_TEST(test_raw_ignores_the_temperature_registers);
    RUN_TEST(test_raw_timestamp_is_in_nanoseconds);
    RUN_TEST(test_raw_i2c_error_is_returned_and_outputs_not_changed);
    RUN_TEST(test_raw_null_timestamp_is_allowed);

    RUN_TEST(test_conv_accel_16384_is_1g);
    RUN_TEST(test_conv_accel_full_scale_is_2g);
    RUN_TEST(test_conv_accel_removes_the_bias);
    RUN_TEST(test_conv_gyro_131_is_1_dps);
    RUN_TEST(test_conv_gyro_full_scale_is_250_dps);
    RUN_TEST(test_conv_gyro_removes_the_bias);
    RUN_TEST(test_conv_does_not_change_the_bias);

    RUN_TEST(test_calib_level_and_still_gives_zero_bias);
    RUN_TEST(test_calib_z_bias_keeps_the_gravity);
    RUN_TEST(test_calib_offsets_of_every_axis);
    RUN_TEST(test_calib_after_calibration_the_sample_reads_0_0_1g);
    RUN_TEST(test_calib_bias_is_the_mean_of_the_samples);
    RUN_TEST(test_calib_uses_200_samples);
    RUN_TEST(test_calib_i2c_error_is_returned_and_bias_not_changed);

    return UNITY_END();
}
