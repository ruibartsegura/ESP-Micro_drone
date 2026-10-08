/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for bmp180.c (host, no hardware). A fake BMP180 is
 *   simulated on the mocked I2C bus: it answers the chip id, has the
 *   calibration EEPROM and, when a conversion command is written in the
 *   control register, it puts the raw temperature / pressure in the data
 *   registers. This way the whole driver is tested, I2C included, not
 *   only the maths.
 *
 *   Reference values: example of the Bosch BMP180 datasheet
 *   (BST-BMP180-DS000, section 3.5): UT = 27898, UP = 23843 (oss = 0)
 *   -> T = 15.0 C, p = 69964 Pa.
 *
 *   The .c file is included directly, so the static functions can be
 *   tested too. Another version can be tested with:
 *       make run T=bmp180 MODULE_SRC=path/to/bmp180.c
 *
 *   Tests that fail with the current code have [BUG] in their comment.
 *
 * Functions:
 *   - fake_bmp_*(): the simulated BMP180.
 *   - setUp() / tearDown(): reset the mocks and load the datasheet example.
 *   - test_init_*: chip id, calibration and errors of bmp180_init().
 *   - test_temp_*: bmp180_read_temperature().
 *   - test_press_*: bmp180_read_pressure() in the 4 oversampling modes.
 *   - test_alt_*: bmp180_pressure_to_altitude().
 *   - main(): runs all the tests.
 */
#include "unity.h"
#include "test_helpers.h"
#include "mocks/mock_reset.h"

#ifndef MODULE_SRC
#define MODULE_SRC "../components/drivers/height/bmp180.c"
#endif
#include MODULE_SRC

#define ADDR BMP180_I2C_ADDR

/* ------------------------------------------------------------------ */
/*                           FAKE BMP180                              */
/* ------------------------------------------------------------------ */
static int32_t fake_ut;      /* raw temperature that the chip will return       */
static int32_t fake_up;      /* raw pressure (already >> (8 - oss)) to return   */

static const int16_t DS_CAL[11] = {
    408,     /* AC1 */
    -72,     /* AC2 */
    -14383,  /* AC3 */
    (int16_t)32741, /* AC4 (unsigned) */
    (int16_t)32757, /* AC5 (unsigned) */
    (int16_t)23153, /* AC6 (unsigned) */
    6190,    /* B1 */
    4,       /* B2 */
    -32768,  /* MB */
    -8711,   /* MC */
    2868     /* MD */
};

static void fake_bmp_load_calibration(const int16_t cal[11]) {
    for (int i = 0; i < 11; i++) mock_i2c_set_be16(ADDR, 0xAA + 2 * i, cal[i]);
}

/* The chip writes the result of the conversion in 0xF6..0xF8. */
static void fake_bmp_write_hook(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len) {
    if (addr != ADDR || reg != 0xF4 || len < 1) return;
    uint8_t cmd = data[0];
    if (cmd == 0x2E) {
        mock_i2c_regs[ADDR][0xF6] = (uint8_t)(fake_ut >> 8);
        mock_i2c_regs[ADDR][0xF7] = (uint8_t)(fake_ut & 0xFF);
    } else if ((cmd & 0x3F) == 0x34) {
        int oss = cmd >> 6;
        uint32_t raw = (uint32_t)fake_up << (8 - oss);   /* 19 bits left aligned in 24 */
        mock_i2c_regs[ADDR][0xF6] = (uint8_t)(raw >> 16);
        mock_i2c_regs[ADDR][0xF7] = (uint8_t)(raw >> 8);
        mock_i2c_regs[ADDR][0xF8] = (uint8_t)(raw & 0xFF);
    }
}

static void fake_bmp_reset(void) {
    mock_i2c_regs[ADDR][0xD0] = 0x55;   /* chip id */
    fake_bmp_load_calibration(DS_CAL);
    fake_ut = 27898;
    fake_up = 23843;
    mock_i2c_write_hook = fake_bmp_write_hook;
}

static bmp180_t dev;

static void init_ok(bmp180_mode_t mode) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, bmp180_init(&dev, I2C_NUM_0, mode));
    mock_i2c_log_len = 0;
    mock_i2c_transactions = 0;
    mock_delay_total_ms = 0;
    mock_delay_calls = 0;
}

/* Index in the I2C log of the n-th write of value v in the control register. */
static int find_ctrl_write(uint8_t value, int nth) {
    for (int i = 0; i < mock_i2c_log_len; i++)
        if (!mock_i2c_log[i].is_read && mock_i2c_log[i].reg == 0xF4 && mock_i2c_log[i].value == value)
            if (nth-- == 0) return i;
    return -1;
}

void setUp(void) {
    mock_reset_all();
    memset(&dev, 0, sizeof dev);
    fake_bmp_reset();
}

void tearDown(void) {}

/* ------------------------------------------------------------------ */
/*                               INIT                                 */
/* ------------------------------------------------------------------ */
void test_init_null_device_is_invalid_arg(void) {
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, bmp180_init(NULL, I2C_NUM_0, BMP180_MODE_STANDARD));
    TEST_ASSERT_EQUAL_INT(0, mock_i2c_transactions);
}

void test_init_ok_with_the_right_chip_id(void) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, bmp180_init(&dev, I2C_NUM_0, BMP180_MODE_STANDARD));
}

void test_init_saves_port_and_mode(void) {
    bmp180_init(&dev, I2C_NUM_1, BMP180_MODE_HIGH_RES);
    TEST_ASSERT_EQUAL_INT(I2C_NUM_1, dev.i2c_port);
    TEST_ASSERT_EQUAL_INT(BMP180_MODE_HIGH_RES, dev.mode);
}

void test_init_reads_the_chip_id_register_first(void) {
    bmp180_init(&dev, I2C_NUM_0, BMP180_MODE_STANDARD);
    TEST_ASSERT_TRUE(mock_i2c_log_len >= 1);
    TEST_ASSERT_TRUE(mock_i2c_log[0].is_read);
    TEST_ASSERT_EQUAL_HEX8(ADDR, mock_i2c_log[0].addr);
    TEST_ASSERT_EQUAL_HEX8(0xD0, mock_i2c_log[0].reg);
}

void test_init_wrong_chip_id_is_not_found(void) {
    mock_i2c_regs[ADDR][0xD0] = 0x58;   /* BMP280 id */
    TEST_ASSERT_EQUAL_INT(ESP_ERR_NOT_FOUND, bmp180_init(&dev, I2C_NUM_0, BMP180_MODE_STANDARD));
}

void test_init_wrong_chip_id_does_not_read_calibration(void) {
    mock_i2c_regs[ADDR][0xD0] = 0x00;
    bmp180_init(&dev, I2C_NUM_0, BMP180_MODE_STANDARD);
    TEST_ASSERT_EQUAL_INT(1, mock_i2c_transactions);
}

void test_init_i2c_error_on_chip_id_is_returned(void) {
    mock_i2c_fail_at = 1;
    mock_i2c_fail_err = ESP_ERR_TIMEOUT;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT, bmp180_init(&dev, I2C_NUM_0, BMP180_MODE_STANDARD));
}

void test_init_i2c_error_on_calibration_is_returned(void) {
    mock_i2c_fail_at = 2;
    mock_i2c_fail_err = ESP_ERR_TIMEOUT;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT, bmp180_init(&dev, I2C_NUM_0, BMP180_MODE_STANDARD));
}

void test_init_reads_22_calibration_bytes_from_0xAA(void) {
    bmp180_init(&dev, I2C_NUM_0, BMP180_MODE_STANDARD);
    TEST_ASSERT_EQUAL_INT(2, mock_i2c_log_len);
    TEST_ASSERT_EQUAL_HEX8(0xAA, mock_i2c_log[1].reg);
    TEST_ASSERT_EQUAL_UINT(22, mock_i2c_log[1].len);
}

void test_init_parses_the_calibration_big_endian(void) {
    bmp180_init(&dev, I2C_NUM_0, BMP180_MODE_STANDARD);
    TEST_ASSERT_EQUAL_INT(408, dev.AC1);
    TEST_ASSERT_EQUAL_INT(-72, dev.AC2);
    TEST_ASSERT_EQUAL_INT(-14383, dev.AC3);
    TEST_ASSERT_EQUAL_UINT(32741, dev.AC4);
    TEST_ASSERT_EQUAL_UINT(32757, dev.AC5);
    TEST_ASSERT_EQUAL_UINT(23153, dev.AC6);
    TEST_ASSERT_EQUAL_INT(6190, dev.B1);
    TEST_ASSERT_EQUAL_INT(4, dev.B2);
    TEST_ASSERT_EQUAL_INT(-32768, dev.MB);
    TEST_ASSERT_EQUAL_INT(-8711, dev.MC);
    TEST_ASSERT_EQUAL_INT(2868, dev.MD);
}

void test_init_unsigned_coefficients_above_32767(void) {
    int16_t cal[11];
    memcpy(cal, DS_CAL, sizeof cal);
    cal[3] = (int16_t)0xFFF0;   /* AC4 = 65520 */
    cal[4] = (int16_t)0x8001;   /* AC5 = 32769 */
    fake_bmp_load_calibration(cal);
    bmp180_init(&dev, I2C_NUM_0, BMP180_MODE_STANDARD);
    TEST_ASSERT_EQUAL_UINT(65520, dev.AC4);
    TEST_ASSERT_EQUAL_UINT(32769, dev.AC5);
}

/* [BUG] The datasheet says that no calibration word can be 0x0000 or
 * 0xFFFF (that means a broken EEPROM or a bus problem). bmp180_init()
 * accepts them, and later a 0 makes a division by zero. */
void test_init_rejects_calibration_words_0x0000(void) {
    int16_t cal[11];
    memcpy(cal, DS_CAL, sizeof cal);
    cal[3] = 0;   /* AC4 */
    fake_bmp_load_calibration(cal);
    TEST_ASSERT_NOT_EQUAL_INT(ESP_OK, bmp180_init(&dev, I2C_NUM_0, BMP180_MODE_STANDARD));
}

/* [BUG] Same as above, with 0xFFFF (what a floating bus returns). */
void test_init_rejects_calibration_words_0xFFFF(void) {
    int16_t cal[11];
    for (int i = 0; i < 11; i++) cal[i] = (int16_t)0xFFFF;
    fake_bmp_load_calibration(cal);
    TEST_ASSERT_NOT_EQUAL_INT(ESP_OK, bmp180_init(&dev, I2C_NUM_0, BMP180_MODE_STANDARD));
}

/* [BUG] The mode is used as an index of delay_ms[4] and as a shift. A wrong
 * value (> 3) must be rejected by bmp180_init(). */
void test_init_rejects_an_invalid_mode(void) {
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, bmp180_init(&dev, I2C_NUM_0, (bmp180_mode_t)7));
}

/* ------------------------------------------------------------------ */
/*                           TEMPERATURE                              */
/* ------------------------------------------------------------------ */
void test_temp_null_arguments_are_invalid_arg(void) {
    float t;
    init_ok(BMP180_MODE_STANDARD);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, bmp180_read_temperature(NULL, &t));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, bmp180_read_temperature(&dev, NULL));
    TEST_ASSERT_EQUAL_INT(0, mock_i2c_transactions);
}

void test_temp_datasheet_example_is_15_0_C(void) {
    float t = -100.0f;
    init_ok(BMP180_MODE_ULTRA_LOW_POWER);
    TEST_ASSERT_EQUAL_INT(ESP_OK, bmp180_read_temperature(&dev, &t));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 15.0f, t);
}

void test_temp_compute_b5_datasheet_value(void) {
    /* The datasheet shows B5 = 2399 because it rounds X2 = -2343.98 to -2344.
     * With C integer division (truncation) X2 = -2343 and B5 = 2400. Both
     * give T = 150 (15.0 C) and p = 69964 Pa, so both are accepted. */
    init_ok(BMP180_MODE_ULTRA_LOW_POWER);
    TEST_ASSERT_INT32_WITHIN(1, 2399, bmp180_compute_b5(&dev, 27898));
}

void test_temp_writes_command_0x2E_in_register_0xF4(void) {
    float t;
    init_ok(BMP180_MODE_STANDARD);
    bmp180_read_temperature(&dev, &t);
    TEST_ASSERT_EQUAL_INT(0x2E, mock_i2c_last_write(ADDR, 0xF4));
}

void test_temp_waits_at_least_4_5_ms_for_the_conversion(void) {
    float t;
    init_ok(BMP180_MODE_STANDARD);
    bmp180_read_temperature(&dev, &t);
    TEST_ASSERT_TRUE(mock_delay_total_ms >= 5);
}

void test_temp_grows_with_the_raw_value(void) {
    /* With the datasheet calibration, UT 23000..38000 is about -42..86 C,
     * the working range of the sensor (-40..85 C). */
    float prev = -1000.0f;
    init_ok(BMP180_MODE_STANDARD);
    for (int32_t ut = 23000; ut <= 38000; ut += 500) {
        float t;
        fake_ut = ut;
        bmp180_read_temperature(&dev, &t);
        TEST_ASSERT_TRUE_MESSAGE(t > prev, "temperature must grow with UT");
        prev = t;
    }
}

void test_temp_resolution_is_0_1_C(void) {
    float t;
    init_ok(BMP180_MODE_STANDARD);
    fake_ut = 28123;
    bmp180_read_temperature(&dev, &t);
    float tenths = t * 10.0f;
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, roundf(tenths), tenths);
}

void test_temp_i2c_error_on_command_is_returned_and_output_not_changed(void) {
    float t = 42.0f;
    init_ok(BMP180_MODE_STANDARD);
    mock_i2c_fail_at = 1;
    TEST_ASSERT_EQUAL_INT(ESP_FAIL, bmp180_read_temperature(&dev, &t));
    TEST_ASSERT_EQUAL_FLOAT(42.0f, t);
}

void test_temp_i2c_error_on_read_is_returned_and_output_not_changed(void) {
    float t = 42.0f;
    init_ok(BMP180_MODE_STANDARD);
    mock_i2c_fail_at = 2;
    TEST_ASSERT_EQUAL_INT(ESP_FAIL, bmp180_read_temperature(&dev, &t));
    TEST_ASSERT_EQUAL_FLOAT(42.0f, t);
}

/* ------------------------------------------------------------------ */
/*                             PRESSURE                               */
/* ------------------------------------------------------------------ */
void test_press_null_arguments_are_invalid_arg(void) {
    int32_t p;
    init_ok(BMP180_MODE_STANDARD);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, bmp180_read_pressure(NULL, &p));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, bmp180_read_pressure(&dev, NULL));
    TEST_ASSERT_EQUAL_INT(0, mock_i2c_transactions);
}

void test_press_datasheet_example_is_69964_Pa(void) {
    int32_t p = 0;
    init_ok(BMP180_MODE_ULTRA_LOW_POWER);
    TEST_ASSERT_EQUAL_INT(ESP_OK, bmp180_read_pressure(&dev, &p));
    TEST_ASSERT_EQUAL_INT32(69964, p);
}

void test_press_reads_temperature_first_and_then_pressure(void) {
    int32_t p;
    init_ok(BMP180_MODE_ULTRA_LOW_POWER);
    bmp180_read_pressure(&dev, &p);
    int i_temp = find_ctrl_write(0x2E, 0);
    int i_press = find_ctrl_write(0x34, 0);
    TEST_ASSERT_TRUE(i_temp >= 0);
    TEST_ASSERT_TRUE(i_press >= 0);
    TEST_ASSERT_TRUE_MESSAGE(i_temp < i_press, "B5 needs the temperature before the pressure");
}

void test_press_reads_3_data_bytes_from_0xF6(void) {
    int32_t p;
    init_ok(BMP180_MODE_STANDARD);
    bmp180_read_pressure(&dev, &p);
    mock_i2c_event_t last = mock_i2c_log[mock_i2c_log_len - 1];
    TEST_ASSERT_TRUE(last.is_read);
    TEST_ASSERT_EQUAL_HEX8(0xF6, last.reg);
    TEST_ASSERT_EQUAL_UINT(3, last.len);
}

/* For each mode: command (0x34 | oss << 6) and conversion time (datasheet table 3). */
static void check_mode(bmp180_mode_t mode, uint8_t cmd, uint64_t min_ms) {
    int32_t p;
    init_ok(mode);
    fake_up = 23843 << mode;   /* same real pressure, more resolution */
    TEST_ASSERT_EQUAL_INT(ESP_OK, bmp180_read_pressure(&dev, &p));
    TEST_ASSERT_TRUE_MESSAGE(find_ctrl_write(cmd, 0) >= 0, "wrong pressure command for the mode");
    /* 5 ms are for the temperature, the rest for the pressure */
    TEST_ASSERT_TRUE_MESSAGE(mock_delay_total_ms >= 5 + min_ms, "conversion time too short for the mode");
    TEST_ASSERT_INT32_WITHIN_MESSAGE(4, 69964, p, "the same pressure must give the same result in every mode");
}

void test_press_mode_ultra_low_power(void) { check_mode(BMP180_MODE_ULTRA_LOW_POWER, 0x34, 5);  }
void test_press_mode_standard(void)        { check_mode(BMP180_MODE_STANDARD,        0x74, 8);  }
void test_press_mode_high_res(void)        { check_mode(BMP180_MODE_HIGH_RES,        0xB4, 14); }
void test_press_mode_ultra_high_res(void)  { check_mode(BMP180_MODE_ULTRA_HIGH_RES,  0xF4, 26); }

void test_press_falls_when_the_raw_value_falls(void) {
    int32_t prev = INT32_MAX;
    init_ok(BMP180_MODE_ULTRA_LOW_POWER);
    for (int32_t up = 40000; up >= 20000; up -= 2000) {
        int32_t p;
        fake_up = up;
        bmp180_read_pressure(&dev, &p);
        TEST_ASSERT_TRUE_MESSAGE(p < prev, "pressure must fall with UP");
        prev = p;
    }
}

void test_press_i2c_error_in_any_step_is_returned(void) {
    /* 4 transactions: temp command, temp read, press command, press read */
    for (int step = 1; step <= 4; step++) {
        int32_t p = 1234;
        init_ok(BMP180_MODE_STANDARD);
        mock_i2c_fail_at = step;
        TEST_ASSERT_EQUAL_INT(ESP_FAIL, bmp180_read_pressure(&dev, &p));
        TEST_ASSERT_EQUAL_INT32_MESSAGE(1234, p, "on error the output must not change");
    }
}

/* [BUG] With a corrupted calibration (AC4 = 0) b4 is 0 and the code divides
 * by zero: the ESP32 crashes. It must return an error instead. */
void test_press_ac4_zero_does_not_crash(void) {
    int32_t p;
    init_ok(BMP180_MODE_STANDARD);
    dev.AC4 = 0;
    esp_err_t ret = ESP_OK;
    TEST_ASSERT_NO_CRASH(ret = bmp180_read_pressure(&dev, &p),
                         "division by zero with AC4 = 0");
    TEST_ASSERT_NOT_EQUAL_INT(ESP_OK, ret);
}

/* [BUG] Same in compute_b5(): x1 + MD == 0 is a division by zero. */
void test_temp_b5_divisor_zero_does_not_crash(void) {
    float t;
    init_ok(BMP180_MODE_STANDARD);
    /* x1 = ((UT - AC6) * AC5) >> 15 ; with UT = AC6, x1 = 0 and MD = 0 */
    fake_ut = dev.AC6;
    dev.MD = 0;
    TEST_ASSERT_NO_CRASH(bmp180_read_temperature(&dev, &t), "division by zero in compute_b5");
}

/* ------------------------------------------------------------------ */
/*                             ALTITUDE                               */
/* ------------------------------------------------------------------ */
void test_alt_zero_at_the_reference_pressure(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, bmp180_pressure_to_altitude(101325, 101325.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, bmp180_pressure_to_altitude(95000, 95000.0f));
}

void test_alt_reference_value_100000_Pa(void) {
    /* Barometric formula: 44330 * (1 - (100000 / 101325)^(1/5.255)) = 110.9 m */
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 110.9f, bmp180_pressure_to_altitude(100000, 101325.0f));
}

void test_alt_datasheet_example(void) {
    /* 69964 Pa is about 3 km over the sea level */
    float h = bmp180_pressure_to_altitude(69964, 101325.0f);
    TEST_ASSERT_FLOAT_WITHIN(15.0f, 3016.0f, h);
}

void test_alt_default_reference_when_not_positive(void) {
    float ref = bmp180_pressure_to_altitude(100000, 101325.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, ref, bmp180_pressure_to_altitude(100000, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, ref, bmp180_pressure_to_altitude(100000, -5.0f));
}

void test_alt_lower_pressure_is_higher_altitude(void) {
    float prev = -1e9f;
    for (int32_t p = 105000; p >= 60000; p -= 1000) {
        float h = bmp180_pressure_to_altitude(p, 101325.0f);
        TEST_ASSERT_TRUE(h > prev);
        prev = h;
    }
}

void test_alt_negative_below_the_reference(void) {
    TEST_ASSERT_TRUE(bmp180_pressure_to_altitude(102000, 101325.0f) < 0.0f);
}

void test_alt_resolution_about_8_cm_per_Pa_near_sea_level(void) {
    /* Near sea level 1 Pa is ~8.4 cm: enough for a drone that flies at 0.5 m */
    float d = bmp180_pressure_to_altitude(101324, 101325.0f) - bmp180_pressure_to_altitude(101325, 101325.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.084f, d);
}

void test_alt_negative_pressure_is_nan(void) {
    TEST_ASSERT_TRUE(isnan(bmp180_pressure_to_altitude(-1, 101325.0f)));
}

/* [BUG] A pressure of 0 Pa is an invalid reading but it returns 44330 m,
 * a value that looks valid. It must return NaN, like a negative pressure. */
void test_alt_zero_pressure_is_nan(void) {
    TEST_ASSERT_TRUE_MESSAGE(isnan(bmp180_pressure_to_altitude(0, 101325.0f)),
                             "0 Pa must not give a valid altitude");
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_init_null_device_is_invalid_arg);
    RUN_TEST(test_init_ok_with_the_right_chip_id);
    RUN_TEST(test_init_saves_port_and_mode);
    RUN_TEST(test_init_reads_the_chip_id_register_first);
    RUN_TEST(test_init_wrong_chip_id_is_not_found);
    RUN_TEST(test_init_wrong_chip_id_does_not_read_calibration);
    RUN_TEST(test_init_i2c_error_on_chip_id_is_returned);
    RUN_TEST(test_init_i2c_error_on_calibration_is_returned);
    RUN_TEST(test_init_reads_22_calibration_bytes_from_0xAA);
    RUN_TEST(test_init_parses_the_calibration_big_endian);
    RUN_TEST(test_init_unsigned_coefficients_above_32767);
    RUN_TEST(test_init_rejects_calibration_words_0x0000);
    RUN_TEST(test_init_rejects_calibration_words_0xFFFF);
    RUN_TEST(test_init_rejects_an_invalid_mode);

    RUN_TEST(test_temp_null_arguments_are_invalid_arg);
    RUN_TEST(test_temp_datasheet_example_is_15_0_C);
    RUN_TEST(test_temp_compute_b5_datasheet_value);
    RUN_TEST(test_temp_writes_command_0x2E_in_register_0xF4);
    RUN_TEST(test_temp_waits_at_least_4_5_ms_for_the_conversion);
    RUN_TEST(test_temp_grows_with_the_raw_value);
    RUN_TEST(test_temp_resolution_is_0_1_C);
    RUN_TEST(test_temp_i2c_error_on_command_is_returned_and_output_not_changed);
    RUN_TEST(test_temp_i2c_error_on_read_is_returned_and_output_not_changed);
    RUN_TEST(test_temp_b5_divisor_zero_does_not_crash);

    RUN_TEST(test_press_null_arguments_are_invalid_arg);
    RUN_TEST(test_press_datasheet_example_is_69964_Pa);
    RUN_TEST(test_press_reads_temperature_first_and_then_pressure);
    RUN_TEST(test_press_reads_3_data_bytes_from_0xF6);
    RUN_TEST(test_press_mode_ultra_low_power);
    RUN_TEST(test_press_mode_standard);
    RUN_TEST(test_press_mode_high_res);
    RUN_TEST(test_press_mode_ultra_high_res);
    RUN_TEST(test_press_falls_when_the_raw_value_falls);
    RUN_TEST(test_press_i2c_error_in_any_step_is_returned);
    RUN_TEST(test_press_ac4_zero_does_not_crash);

    RUN_TEST(test_alt_zero_at_the_reference_pressure);
    RUN_TEST(test_alt_reference_value_100000_Pa);
    RUN_TEST(test_alt_datasheet_example);
    RUN_TEST(test_alt_default_reference_when_not_positive);
    RUN_TEST(test_alt_lower_pressure_is_higher_altitude);
    RUN_TEST(test_alt_negative_below_the_reference);
    RUN_TEST(test_alt_resolution_about_8_cm_per_Pa_near_sea_level);
    RUN_TEST(test_alt_negative_pressure_is_nan);
    RUN_TEST(test_alt_zero_pressure_is_nan);

    return UNITY_END();
}
