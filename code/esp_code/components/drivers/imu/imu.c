/**
 * Made by Rui B.S.
 * Date: 23/05/2026
 * email: rui.bartolome@gmail.com
 *
 */
#include "imu.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#define DEG_TO_RAD (float)(M_PI / 180.0)

// Frecuencia de muestreo/filtrado interna, independiente de a que
// ritmo se publique por ROS. 100Hz es un buen punto de partida para
// un Madgwick con MPU6050 en un dron pequeno.
#define IMU_TASK_PERIOD_MS 10

static bool is_init = false;

// Bias de calibracion (constantes tras imu_init(), no necesitan lock)
static float accel_bias[3] = {0.0f, 0.0f, 0.0f};
static float gyro_bias[3]  = {0.0f, 0.0f, 0.0f};

// Ultimo resultado calculado, compartido entre imu_task() (escritor)
// y cualquier tarea que llame a imu_get_data() (lectores). Protegido
// por una seccion critica corta (solo copia de struct, sin I/O).
static IMU latest_data;
static portMUX_TYPE data_mux = portMUX_INITIALIZER_UNLOCKED;

static void imu_sample_and_filter(void)
{
    esp_err_t ret;
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    int64_t t_stamp;

    // Get the data from the sensor
    ret = mpu6050_read_raw_data(I2C_MASTER_NUM,
                                 &accel_x, &accel_y, &accel_z,
                                 &gyro_x, &gyro_y, &gyro_z, &t_stamp);
    if (ret != ESP_OK) {
        return; // se conserva el ultimo dato valido, no se pisa con basura
    }

    // Convert the data from raw to units(g)
    mpu6050_convert_accel(accel_x, accel_y, accel_z, accel_bias,
                           &accel_x_g, &accel_y_g, &accel_z_g);
    mpu6050_convert_gyro(gyro_x, gyro_y, gyro_z, gyro_bias,
                          &gyro_x_dps, &gyro_y_dps, &gyro_z_dps);


    // Fill the struct with the data
    IMU sample = {
        .AcX_g = accel_x_g,
        .AcY_g = accel_y_g,
        .AcZ_g = accel_z_g,
        .GyX_dps = gyro_x_dps,
        .GyY_dps = gyro_y_dps,
        .GyZ_dps = gyro_z_dps,
        .Time_stamp = t_stamp,
    };

    portENTER_CRITICAL(&data_mux);
    latest_data = sample;
    portEXIT_CRITICAL(&data_mux);
}

static void imu_task(void *arg) {
    (void)arg;

    int log_counter = 0;

    while (1) {
        imu_sample_and_filter();

        if (++log_counter >= 500) {
            log_counter = 0;
            UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI("imu_task", "stack libre (min historico): %u bytes",
                     (unsigned)(free_words * sizeof(StackType_t)));
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }
}

void imu_init(void) {
    if (is_init) {
        return;
    }

    esp_err_t ret;

    // Initialize I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        return;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, ESP_INTR_FLAG_DEFAULT);
    if (ret != ESP_OK) {
        return;
    }

    // Initialize MPU6050
    ret = mpu6050_init(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        return;
    }

    // Calibrate the MPU6050 (el dron debe estar quieto y nivelado aqui)
    ret = mpu6050_calibrate(I2C_MASTER_NUM, accel_bias, gyro_bias);
    
    // If everything is ok, continue
    if (ret != ESP_OK) {
        return;
    }

    // Task to read the IMU
    xTaskCreate(imu_task, "imu_task", CONFIG_IMU_TASK_STACK, NULL, CONFIG_IMU_TASK_PRIO, NULL);

    is_init = true;
}

bool imu_test(void)
{
    return is_init;
}

esp_err_t imu_get_data(IMU *data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_init) {
        return ESP_ERR_INVALID_STATE;
    }

    // Fill the data
    portENTER_CRITICAL(&data_mux);
    *data = latest_data;
    portEXIT_CRITICAL(&data_mux);

    return ESP_OK;
}


esp_err_t imu_check_stable_for_arming(imu_arm_state_t *state) {
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_init) {
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < IMU_ARM_CHECK_SAMPLES; i++) {
        IMU sample;
        esp_err_t ret = imu_get_data(&sample);
        if (ret != ESP_OK) {
            return ret;
        }

        float gyro_mag = sqrtf(sample.GyX_dps * sample.GyX_dps +
                                sample.GyY_dps * sample.GyY_dps +
                                sample.GyZ_dps * sample.GyZ_dps);
        if (gyro_mag > IMU_ARM_MAX_GYRO_DPS) {
            *state = IMU_ARM_STATE_MOVING;
            return ESP_OK;
        }

        float accel_mag = sqrtf(sample.AcX_g * sample.AcX_g +
                                 sample.AcY_g * sample.AcY_g +
                                 sample.AcZ_g * sample.AcZ_g);
        if (accel_mag < IMU_ARM_ACCEL_MIN_G || accel_mag > IMU_ARM_ACCEL_MAX_G) {
            *state = IMU_ARM_STATE_ACCEL_ABNORMAL;
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_ARM_CHECK_PERIOD_MS));
    }

    *state = IMU_ARM_STATE_STABLE;
    return ESP_OK;
}