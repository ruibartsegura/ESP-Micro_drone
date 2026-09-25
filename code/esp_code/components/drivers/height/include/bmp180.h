/*
 * bmp180.h
 *
 * Driver minimo para el sensor de presion/temperatura BMP180 sobre I2C,
 * pensado para ESP32-S2 con ESP-IDF (driver I2C "legacy": driver/i2c.h).
 *
 * Pensado para el proyecto de micro-dron: el GY-87 trae un BMP180 (ademas
 * del MPU6050 y el HMC5883L), asi que este driver solo cubre la parte
 * de presion/temperatura del modulo.
 */

#ifndef BMP180_H
#define BMP180_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// Direccion I2C fija del BMP180 (7 bits)
#define BMP180_I2C_ADDR 0x77

// Modo de sobremuestreo de presion (OSS). A mayor valor, mas precision
// pero mas tiempo de conversion.
typedef enum {
    BMP180_MODE_ULTRA_LOW_POWER = 0, // 4.5 ms
    BMP180_MODE_STANDARD        = 1, // 7.5 ms
    BMP180_MODE_HIGH_RES        = 2, // 13.5 ms
    BMP180_MODE_ULTRA_HIGH_RES  = 3, // 25.5 ms
} bmp180_mode_t;

// Handle con el puerto I2C y los coeficientes de calibracion leidos del sensor
typedef struct {
    i2c_port_t i2c_port;
    bmp180_mode_t mode;

    // Coeficientes de calibracion (EEPROM interna del sensor, se leen una
    // sola vez en el init)
    int16_t AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t B1, B2;
    int16_t MB, MC, MD;
} bmp180_t;

/**
 * @brief Inicializa el sensor: comprueba el chip id y lee los coeficientes
 *        de calibracion.
 *
 * @param dev        Puntero a la estructura de handle (la rellena esta funcion)
 * @param i2c_port   Puerto I2C ya configurado e instalado (i2c_param_config +
 *                   i2c_driver_install) por el llamador
 * @param mode       Modo de sobremuestreo a usar en las lecturas de presion
 */
esp_err_t bmp180_init(bmp180_t *dev, i2c_port_t i2c_port, bmp180_mode_t mode);

/**
 * @brief Lee la temperatura compensada en grados Celsius.
 */
esp_err_t bmp180_read_temperature(bmp180_t *dev, float *temperature_c);

/**
 * @brief Lee la presion compensada en Pascales.
 *
 * Internamente lee tambien la temperatura, porque la formula de
 * compensacion de presion del BMP180 la necesita.
 */
esp_err_t bmp180_read_pressure(bmp180_t *dev, int32_t *pressure_pa);

/**
 * @brief Calcula la altitud aproximada en metros a partir de la presion
 *        medida y una presion de referencia a nivel del mar (por defecto
 *        101325 Pa).
 */
float bmp180_pressure_to_altitude(int32_t pressure_pa, float sea_level_pa);

#ifdef __cplusplus
}
#endif

#endif // BMP180_H