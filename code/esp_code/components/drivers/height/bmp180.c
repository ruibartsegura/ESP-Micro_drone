/*
 * bmp180.c
 *
 * Implementacion del driver BMP180 (I2C, ESP-IDF driver/i2c.h legacy).
 * Formulas de compensacion tomadas del datasheet oficial Bosch BMP180.
 */

#include "bmp180.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "BMP180";

// BMP180 registers
#define BMP180_REG_CHIP_ID      0xD0
#define BMP180_CHIP_ID_VALUE    0x55

#define BMP180_REG_CAL_START    0xAA // Start of 22 calibration bytes (11 x int16)
#define BMP180_REG_CTRL         0xF4
#define BMP180_REG_DATA_MSB     0xF6

// Datasheet cmd to read temperature and presure
#define BMP180_CMD_READ_TEMP    0x2E
#define BMP180_CMD_READ_PRESS   0x34

#define I2C_TIMEOUT_MS 100

// ---------- Helpers de bajo nivel I2C ----------

static esp_err_t bmp180_write_reg(bmp180_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_write_to_device(dev->i2c_port, BMP180_I2C_ADDR, buf,
                                       sizeof(buf),
                                       pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t bmp180_read_regs(bmp180_t *dev, uint8_t reg, uint8_t *data,
                                   size_t len)
{
    return i2c_master_write_read_device(dev->i2c_port, BMP180_I2C_ADDR,
                                         &reg, 1, data, len,
                                         pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

// Parser from Big Endian
static inline int16_t be16(const uint8_t *p)
{
    return (int16_t)((p[0] << 8) | p[1]);
}

// ---------- API publica ----------

esp_err_t bmp180_init(bmp180_t *dev, i2c_port_t i2c_port, bmp180_mode_t mode)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_port = i2c_port;
    dev->mode = mode;

    // 1. Comprobar chip id
    uint8_t chip_id = 0;
    esp_err_t err = bmp180_read_regs(dev, BMP180_REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo leyendo chip id: %s", esp_err_to_name(err));
        return err;
    }
    if (chip_id != BMP180_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "Chip id inesperado: 0x%02X (se esperaba 0x%02X)",
                 chip_id, BMP180_CHIP_ID_VALUE);
        return ESP_ERR_NOT_FOUND;
    }

    // 2. Leer los 22 bytes (11 palabras de 16 bits) de calibracion
    uint8_t cal[22];
    err = bmp180_read_regs(dev, BMP180_REG_CAL_START, cal, sizeof(cal));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo leyendo calibracion: %s", esp_err_to_name(err));
        return err;
    }

    dev->AC1 = be16(&cal[0]);
    dev->AC2 = be16(&cal[2]);
    dev->AC3 = be16(&cal[4]);
    dev->AC4 = (uint16_t)be16(&cal[6]);
    dev->AC5 = (uint16_t)be16(&cal[8]);
    dev->AC6 = (uint16_t)be16(&cal[10]);
    dev->B1  = be16(&cal[12]);
    dev->B2  = be16(&cal[14]);
    dev->MB  = be16(&cal[16]);
    dev->MC  = be16(&cal[18]);
    dev->MD  = be16(&cal[20]);

    ESP_LOGI(TAG, "BMP180 inicializado correctamente (chip id 0x%02X)", chip_id);
    return ESP_OK;
}

// Lee el valor crudo de temperatura (UT) sin compensar
static esp_err_t bmp180_read_raw_temp(bmp180_t *dev, int32_t *ut)
{
    esp_err_t err = bmp180_write_reg(dev, BMP180_REG_CTRL, BMP180_CMD_READ_TEMP);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(5)); // datasheet: 4.5 ms

    uint8_t data[2];
    err = bmp180_read_regs(dev, BMP180_REG_DATA_MSB, data, 2);
    if (err != ESP_OK) return err;

    *ut = (int32_t)((data[0] << 8) | data[1]);
    return ESP_OK;
}

// Lee el valor crudo de presion (UP) sin compensar, segun el OSS configurado
static esp_err_t bmp180_read_raw_pressure(bmp180_t *dev, int32_t *up)
{
    uint8_t cmd = BMP180_CMD_READ_PRESS | (dev->mode << 6);
    esp_err_t err = bmp180_write_reg(dev, BMP180_REG_CTRL, cmd);
    if (err != ESP_OK) return err;

    // Tiempo de conversion segun el modo (datasheet, tabla 3)
    static const uint8_t delay_ms[4] = { 5, 8, 14, 26 };
    vTaskDelay(pdMS_TO_TICKS(delay_ms[dev->mode]));

    uint8_t data[3];
    err = bmp180_read_regs(dev, BMP180_REG_DATA_MSB, data, 3);
    if (err != ESP_OK) return err;

    *up = (int32_t)(((data[0] << 16) | (data[1] << 8) | data[2])
                     >> (8 - dev->mode));
    return ESP_OK;
}

// Calcula B5 (valor intermedio compartido por temperatura y presion)
static int32_t bmp180_compute_b5(bmp180_t *dev, int32_t ut)
{
    int32_t x1 = ((ut - (int32_t)dev->AC6) * (int32_t)dev->AC5) >> 15;
    int32_t x2 = ((int32_t)dev->MC << 11) / (x1 + dev->MD);
    return x1 + x2;
}

esp_err_t bmp180_read_temperature(bmp180_t *dev, float *temperature_c)
{
    if (dev == NULL || temperature_c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t ut;
    esp_err_t err = bmp180_read_raw_temp(dev, &ut);
    if (err != ESP_OK) return err;

    int32_t b5 = bmp180_compute_b5(dev, ut);
    int32_t t = (b5 + 8) >> 4; // decimas de grado

    *temperature_c = t / 10.0f;
    return ESP_OK;
}

esp_err_t bmp180_read_pressure(bmp180_t *dev, int32_t *pressure_pa)
{
    if (dev == NULL || pressure_pa == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t ut, up;
    esp_err_t err = bmp180_read_raw_temp(dev, &ut);
    if (err != ESP_OK) return err;

    err = bmp180_read_raw_pressure(dev, &up);
    if (err != ESP_OK) return err;

    int32_t b5 = bmp180_compute_b5(dev, ut);

    int32_t b6 = b5 - 4000;
    int32_t x1 = ((int32_t)dev->B2 * ((b6 * b6) >> 12)) >> 11;
    int32_t x2 = ((int32_t)dev->AC2 * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = ((((int32_t)dev->AC1 * 4 + x3) << dev->mode) + 2) >> 2;

    x1 = ((int32_t)dev->AC3 * b6) >> 13;
    x2 = ((int32_t)dev->B1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = ((uint32_t)dev->AC4 * (uint32_t)(x3 + 32768)) >> 15;
    uint32_t b7 = ((uint32_t)(up - b3)) * (50000UL >> dev->mode);

    int32_t p;
    if (b7 < 0x80000000UL) {
        p = (b7 * 2) / b4;
    } else {
        p = (b7 / b4) * 2;
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);

    *pressure_pa = p;
    return ESP_OK;
}

float bmp180_pressure_to_altitude(int32_t pressure_pa, float sea_level_pa)
{
    if (sea_level_pa <= 0.0f) {
        sea_level_pa = 101325.0f;
    }
    // Barometric formula
    return 44330.0f * (1.0f - powf((float)pressure_pa / sea_level_pa, 1.0f / 5.255f));
}