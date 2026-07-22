/********************************************************************************************
 * Project: MPU6050 ESP32 Sensor Interface
 * Original author: Muhammad Idrees
 *
 * CORRECCIONES aplicadas (ver comentarios "FIX:"):
 *  1. mpu6050_init() no escribía el registro PWR_MGMT_1 realmente.
 *  2. El bias calculado en mpu6050_calibrate() se descartaba (shadowing de
 *     variables estáticas del módulo). Ahora el bias se pasa siempre
 *     explícito, no hay estado global oculto.
 *  3. Se configura explícitamente el rango de ±2g / ±250°/s en vez de
 *     confiar en el valor de reset del chip.
 *  4. Lectura de registros con transacción atómica (write+read con
 *     repeated start) en vez de dos transacciones I2C separadas.
 ********************************************************************************************/

#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ACCEL_SCALE 16384.0f // LSB/g para ±2g
#define GYRO_SCALE  131.0f   // LSB/(°/s) para ±250°/s

static esp_err_t mpu6050_write_reg(i2c_port_t i2c_num, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_write_to_device(i2c_num, MPU6050_ADDR, buf, sizeof(buf),
                                       1000 / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_init(i2c_port_t i2c_num)
{
    esp_err_t ret;

    // FIX 1: antes se escribía un byte suelto (0x00) sin indicar el
    // registro destino, así que nunca se limpiaba el bit SLEEP de
    // PWR_MGMT_1 de forma fiable. Ahora se escribe [registro, valor].
    ret = mpu6050_write_reg(i2c_num, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }

    // Dar tiempo al sensor a estabilizar tras salir de sleep
    vTaskDelay(pdMS_TO_TICKS(50));

    // FIX 3: fijar explícitamente el rango, en vez de asumir el valor
    // de reset del chip (que coincide con ACCEL_SCALE/GYRO_SCALE, pero
    // conviene no depender de eso).
    ret = mpu6050_write_reg(i2c_num, MPU6050_REG_GYRO_CONFIG, 0x00);  // ±250 °/s
    if (ret != ESP_OK) {
        return ret;
    }

    ret = mpu6050_write_reg(i2c_num, MPU6050_REG_ACCEL_CONFIG, 0x00); // ±2 g
    if (ret != ESP_OK) {
        return ret;
    }

    // Filtro paso-bajo digital (~44 Hz accel / 42 Hz gyro) para reducir
    // ruido de vibración de motores/hélices.
    ret = mpu6050_write_reg(i2c_num, MPU6050_REG_CONFIG, 0x03);
    if (ret != ESP_OK) {
        return ret;
    }

    // Sample rate = 1kHz / (1 + SMPLRT_DIV). Con DLPF activo el sample
    // rate base es 1kHz; div=0 -> 1kHz de muestreo interno.
    ret = mpu6050_write_reg(i2c_num, MPU6050_REG_SMPLRT_DIV, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t mpu6050_read_raw_data(i2c_port_t i2c_num,
                                 int16_t *accel_x, int16_t *accel_y, int16_t *accel_z,
                                 int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
    uint8_t data[14];
    esp_err_t ret;
    uint8_t reg_addr = MPU6050_REG_ACCEL_XOUT_H;

    // FIX 4: una sola transacción atómica (repeated start) en vez de
    // write + read separados. Evita que otro maestro/tarea intercale
    // una transacción entre medias y quedarnos leyendo el registro
    // equivocado.
    ret = i2c_master_write_read_device(i2c_num, MPU6050_ADDR,
                                        &reg_addr, 1,
                                        data, sizeof(data),
                                        1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        return ret;
    }

    *accel_x = (int16_t)((data[0] << 8) | data[1]);
    *accel_y = (int16_t)((data[2] << 8) | data[3]);
    *accel_z = (int16_t)((data[4] << 8) | data[5]);
    // data[6],data[7] = temperatura, se ignora
    *gyro_x  = (int16_t)((data[8]  << 8) | data[9]);
    *gyro_y  = (int16_t)((data[10] << 8) | data[11]);
    *gyro_z  = (int16_t)((data[12] << 8) | data[13]);

    return ESP_OK;
}

void mpu6050_convert_accel(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                            const float bias[3],
                            float *accel_x, float *accel_y, float *accel_z)
{
    // FIX 2 (unidades): salida en g "puras", NO en m/s². El campo se
    // llama AcX_g y el chequeo de armado (imu_check_stable_for_arming)
    // compara contra 0.9-1.1 asumiendo g. La conversión a m/s² para el
    // mensaje ROS se hace aparte, en ros_coordinator.c::fill_imu_msg().
    *accel_x = (raw_x / ACCEL_SCALE) - bias[0];
    *accel_y = (raw_y / ACCEL_SCALE) - bias[1];
    *accel_z = (raw_z / ACCEL_SCALE) - bias[2];
}

void mpu6050_convert_gyro(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                           const float bias[3],
                           float *gyro_x, float *gyro_y, float *gyro_z)
{
    *gyro_x = (raw_x / GYRO_SCALE) - bias[0];
    *gyro_y = (raw_y / GYRO_SCALE) - bias[1];
    *gyro_z = (raw_z / GYRO_SCALE) - bias[2];
}

esp_err_t mpu6050_calibrate(i2c_port_t i2c_num, float *accel_bias, float *gyro_bias)
{
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    float accel_x_sum = 0.0f, accel_y_sum = 0.0f, accel_z_sum = 0.0f;
    float gyro_x_sum = 0.0f, gyro_y_sum = 0.0f, gyro_z_sum = 0.0f;
    const int samples = 200;
    const float zero_bias[3] = { 0.0f, 0.0f, 0.0f };

    // El dron DEBE estar quieto y nivelado durante esta rutina.
    for (int i = 0; i < samples; i++) {
        esp_err_t ret = mpu6050_read_raw_data(i2c_num, &accel_x, &accel_y, &accel_z,
                                               &gyro_x, &gyro_y, &gyro_z);
        if (ret != ESP_OK) {
            return ret; // antes se ignoraba el error de lectura
        }

        // Convertimos sin bias (bias=0) para medir el offset crudo real.
        mpu6050_convert_accel(accel_x, accel_y, accel_z, zero_bias,
                               &accel_x_g, &accel_y_g, &accel_z_g);
        mpu6050_convert_gyro(gyro_x, gyro_y, gyro_z, zero_bias,
                              &gyro_x_dps, &gyro_y_dps, &gyro_z_dps);

        accel_x_sum += accel_x_g;
        accel_y_sum += accel_y_g;
        accel_z_sum += accel_z_g;
        gyro_x_sum  += gyro_x_dps;
        gyro_y_sum  += gyro_y_dps;
        gyro_z_sum  += gyro_z_dps;

        vTaskDelay(pdMS_TO_TICKS(5)); // dejar respirar al bus I2C
    }

    accel_bias[0] = accel_x_sum / samples;
    accel_bias[1] = accel_y_sum / samples;
    accel_bias[2] = accel_z_sum / samples - 1.0f; // en reposo Z debe leer +1g

    gyro_bias[0] = gyro_x_sum / samples;
    gyro_bias[1] = gyro_y_sum / samples;
    gyro_bias[2] = gyro_z_sum / samples;

    return ESP_OK;
}