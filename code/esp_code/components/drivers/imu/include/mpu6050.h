/**
 * Made by Rui B.S.
 * Date: 22/07/2026
 * email: rui.bartolome@gmail.com
 * Based on: "MPU6050 ESP32 Sensor Interface", original author Muhammad Idrees
 *
 * Description:
 *   Public interface and register addresses of the MPU-6050 driver.
 *   Changes: init fix and explicit bias (no hidden global state).
 *
 * Functions:
 *   - mpu6050_init(): wakes up and configures the sensor.
 *   - mpu6050_read_raw_data(): reads the raw accelerometer and gyroscope data.
 *   - mpu6050_convert_accel() / mpu6050_convert_gyro(): convert raw data to units.
 *   - mpu6050_calibrate(): computes the bias. The drone must be still and level.
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"
#include "esp_err.h"

#define MPU6050_ADDR 0x68

// MPU6050 Register Addresses
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43

// Function prototypes
esp_err_t mpu6050_init(i2c_port_t i2c_num);
esp_err_t mpu6050_read_raw_data(i2c_port_t i2c_num,
                                 int16_t *accel_x, int16_t *accel_y, int16_t *accel_z,
                                 int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z,
                                 int64_t *t_stamp);

// NOTA: bias ahora es un parámetro explícito. Ya no hay bias "fantasma"
// escondido en una variable estática que la calibración nunca actualizaba.
void mpu6050_convert_accel(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                            const float bias[3],
                            float *accel_x, float *accel_y, float *accel_z);
void mpu6050_convert_gyro(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                           const float bias[3],
                           float *gyro_x, float *gyro_y, float *gyro_z);

// Calcula accel_bias[3] y gyro_bias[3] (salidas). El dron debe estar
// quieto y nivelado mientras se ejecuta.
esp_err_t mpu6050_calibrate(i2c_port_t i2c_num, float *accel_bias, float *gyro_bias);

#endif // MPU6050_H
