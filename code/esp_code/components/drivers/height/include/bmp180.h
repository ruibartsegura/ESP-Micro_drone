/**
 * Made by Rui B.S.
 * Date: 25/09/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Public interface of the BMP180 driver (pressure and temperature over
 *   I2C, legacy ESP-IDF driver). The drone uses the GY-87 board, which has
 *   a BMP180, an MPU6050 and an HMC5883L. This driver only covers the
 *   pressure and temperature part.
 *
 * Functions:
 *   - bmp180_init(): checks the chip id and reads the calibration coefficients.
 *   - bmp180_read_temperature(): returns the compensated temperature (C).
 *   - bmp180_read_pressure(): returns the compensated pressure (Pa).
 *   - bmp180_pressure_to_altitude(): converts pressure to altitude (m).
 */

#ifndef BMP180_H
#define BMP180_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// Fixed I2C address of the BMP180 (7 bits)
#define BMP180_I2C_ADDR 0x77

// Pressure oversampling mode (OSS). A higher value gives more precision
// but a longer conversion time.
typedef enum {
    BMP180_MODE_ULTRA_LOW_POWER = 0, // 4.5 ms
    BMP180_MODE_STANDARD        = 1, // 7.5 ms
    BMP180_MODE_HIGH_RES        = 2, // 13.5 ms
    BMP180_MODE_ULTRA_HIGH_RES  = 3, // 25.5 ms
} bmp180_mode_t;

// Handle with the I2C port and the calibration coefficients read from the sensor
typedef struct {
    i2c_port_t i2c_port;
    bmp180_mode_t mode;

    // Calibration coefficients (internal EEPROM of the sensor, read only
    // once in the init)
    int16_t AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t B1, B2;
    int16_t MB, MC, MD;
} bmp180_t;


// Initialises the sensor: checks the chip id and reads the
// calibration coefficients.
esp_err_t bmp180_init(bmp180_t *dev, i2c_port_t i2c_port, bmp180_mode_t mode);

// Reads the compensated temperature in degrees Celsius.
esp_err_t bmp180_read_temperature(bmp180_t *dev, float *temperature_c);


// Reads the compensated pressure in Pascals.
// It also reads the temperature, because the pressure compensation
// formula of the BMP180 needs it.
esp_err_t bmp180_read_pressure(bmp180_t *dev, int32_t *pressure_pa);


// Computes the approximate altitude in metres from the measured
// pressure and a sea level reference pressure (101325 Pa by
// default). Returns NaN if the pressure is <= 0.
float bmp180_pressure_to_altitude(float pressure_pa, float sea_level_pa);

#ifdef __cplusplus
}
#endif

#endif // BMP180_H