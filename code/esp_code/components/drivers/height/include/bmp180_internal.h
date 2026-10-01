/*
 * bmp180_internal.h
 *
 * Declaraciones SOLO para tests. No forman parte de la API publica del
 * driver (bmp180.h no las expone) -- son los pasos internos de compensacion
 * de la Figura 4 del datasheet Bosch BMP180, separados de la lectura I2C
 * para poder testearlos sin sensor fisico ni ESP32 conectado.
 *
 * No incluir este header desde codigo de aplicacion (height.c, main.c...),
 * solo desde test/test_bmp180.c.
 */

#ifndef BMP180_INTERNAL_H
#define BMP180_INTERNAL_H

#include "bmp180.h"

#ifdef __cplusplus
extern "C" {
#endif

// Paso 1 del datasheet (bmp180_get_ut -> B5): valor intermedio compartido
// por la compensacion de temperatura y de presion.
int32_t bmp180_compute_b5(const bmp180_t *dev, int32_t ut);

// Paso 2a (BMP180_get_temperature): T = (B5+8)>>4, en decimas de grado,
// convertido aqui a grados Celsius en temperature_c.
void bmp180_compensate_temperature(int32_t b5, float *temperature_c);

// Paso 2b (BMP180_calpressure): presion compensada en Pascales.
// Devuelve ESP_ERR_INVALID_STATE si b4 calcularia una division por cero
// (coeficientes de calibracion invalidos).
esp_err_t bmp180_compensate_pressure(const bmp180_t *dev, int32_t b5,
                                      int32_t up, int32_t *pressure_pa);

#ifdef __cplusplus
}
#endif

#endif // BMP180_INTERNAL_H
