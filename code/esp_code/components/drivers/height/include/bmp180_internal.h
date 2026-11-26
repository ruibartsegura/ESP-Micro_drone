/**
 * Made by Rui B.S.
 * Date: 01/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Declarations ONLY for tests. They are not part of the public API of the
 *   driver. They are the internal compensation steps of Figure 4 of the
 *   Bosch BMP180 datasheet, split from the I2C reading so they can be tested
 *   without the sensor. Include it only from test/test_bmp180.c, never from
 *   application code.
 *
 * Functions:
 *   - bmp180_compute_b5(): step 1, computes the B5 value.
 *   - bmp180_compensate_temperature(): step 2a, compensated temperature (C).
 *   - bmp180_compensate_pressure(): step 2b, compensated pressure (Pa).
 */

#ifndef BMP180_INTERNAL_H
#define BMP180_INTERNAL_H

#include "bmp180.h"

#ifdef __cplusplus
extern "C" {
#endif

// Datasheet step 1 (bmp180_get_ut -> B5): intermediate value shared by
// the temperature and pressure compensation.
int32_t bmp180_compute_b5(const bmp180_t *dev, int32_t ut);

// Step 2a (BMP180_get_temperature): T = (B5+8)>>4, in tenths of a degree,
// converted here to degrees Celsius in temperature_c.
void bmp180_compensate_temperature(int32_t b5, float *temperature_c);

// Step 2b (BMP180_calpressure): compensated pressure in Pascals.
// Returns ESP_ERR_INVALID_STATE if b4 would give a division by zero
// (invalid calibration coefficients).
esp_err_t bmp180_compensate_pressure(const bmp180_t *dev, int32_t b5,
                                      int32_t up, int32_t *pressure_pa);

#ifdef __cplusplus
}
#endif

#endif // BMP180_INTERNAL_H
