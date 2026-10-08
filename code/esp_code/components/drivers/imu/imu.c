/**
 * Made by Rui B.S.
 * Date: 23/05/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   IMU module. It starts the I2C bus and the MPU-6050, calibrates it and
 *   reads it in its own FreeRTOS task every 10 ms. The data (acceleration
 *   in g and angular velocity in deg/s) is saved in the global state.
 *   In simulation mode the sensor is not used.
 *
 * Functions:
 *   - imu_check_stable_for_arming(): checks if the drone is still and level, so it is safe to arm.
 *   - imu_sample_and_filter(): reads the sensor and saves the data in the global state.
 *   - imu_task(): FreeRTOS task that reads the IMU periodically.
 *   - imu_init(): starts the I2C bus, the sensor, the calibration and the task.
 *   - imu_test(): checks that the module started correctly.
 */

#include "imu.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "state.h"

#define DEG_TO_RAD (float)(M_PI / 180.0)

// Frecuencia de muestreo/filtrado interna, independiente de a que
// ritmo se publique por ROS. 100Hz es un buen punto de partida para
// un Madgwick con MPU6050 en un dron pequeno.
#define IMU_TASK_PERIOD_MS 10

static bool is_init = false;

// With simulation skip this params
#ifndef CONFIG_SIMULATION_ON
    // Bias de calibracion (constantes tras imu_init(), no necesitan lock)
    static float accel_bias[3] = {0.0f, 0.0f, 0.0f};
    static float gyro_bias[3]  = {0.0f, 0.0f, 0.0f};

#endif

static portMUX_TYPE data_mux = portMUX_INITIALIZER_UNLOCKED;


esp_err_t imu_check_stable_for_arming(imu_arm_state_t *state) {
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_init) {
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < IMU_ARM_CHECK_SAMPLES; i++) {
        // Last sample of the global state (g and deg/s), written by imu_task
        vec3_t acc, vel;
        get_acc_lin(&acc);
        get_vel_ang(&vel);

        float gyro_mag = sqrtf(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
        if (gyro_mag > IMU_ARM_MAX_GYRO_DPS) {
            *state = IMU_ARM_STATE_MOVING;
            return ESP_OK;
        }

        float accel_mag = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
        if (accel_mag < IMU_ARM_ACCEL_MIN_G || accel_mag > IMU_ARM_ACCEL_MAX_G) {
            *state = IMU_ARM_STATE_ACCEL_ABNORMAL;
            return ESP_OK;
        }

        // Angle between the gravity and the Z axis of the drone
        float tilt_deg = acosf(acc.z / accel_mag) / DEG_TO_RAD;
        if (tilt_deg > IMU_ARM_MAX_TILT_DEG) {
            *state = IMU_ARM_STATE_TILTED;
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_ARM_CHECK_PERIOD_MS));
    }

    *state = IMU_ARM_STATE_STABLE;
    return ESP_OK;
}


static void imu_sample_and_filter(void)
{
    #ifndef CONFIG_SIMULATION_ON
        esp_err_t ret;
        int16_t accel_x, accel_y, accel_z;
        int16_t gyro_x, gyro_y, gyro_z;
        float accel_lin_x, accel_lin_y, accel_lin_z;
        float vel_ang_x, vel_ang_y, vel_ang_z;
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
                            &accel_lin_x, &accel_lin_y, &accel_lin_z);
        mpu6050_convert_gyro(gyro_x, gyro_y, gyro_z, gyro_bias,
                            &vel_ang_x, &vel_ang_y, &vel_ang_z);

        // Load data to the global data
        set_vel_ang(vel_ang_x, vel_ang_y, vel_ang_z);
        set_acc_lin(accel_lin_x, accel_lin_y, accel_lin_z);
        set_time_imu(t_stamp);
    #endif
}

static void imu_task(void *arg) {
    (void)arg;

    int log_counter = 0;

    while (1) {
        imu_sample_and_filter();

        if (++log_counter >= 500) {
            log_counter = 0;
            UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
            // // ESP_LOGI("imu_task", "stack libre (min historico): %u bytes",
            //          (unsigned)(free_words * sizeof(StackType_t)));
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }
}


void imu_init(void) {
    if (is_init) {
        return;
    }

    esp_err_t ret;

    // With simulation skip sensor driver initialization
    #ifndef CONFIG_SIMULATION_ON
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
    #endif


    // Task to read the IMU
    xTaskCreate(imu_task, "imu_task", CONFIG_IMU_TASK_STACK, NULL, CONFIG_IMU_TASK_PRIO, NULL);

    is_init = true;
}

bool imu_test(void)
{
    return is_init;
}
