/**
 * Made by Rui B.S.
 * Date: 25/09/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Height module. It starts the I2C bus and the BMP180 barometer and reads
 *   it in its own FreeRTOS task every 10 ms. The pressure is converted to
 *   altitude and saved in the global state. In simulation mode the sensor
 *   is not used.
 *
 * Functions:
 *   - i2c_master_init(): starts the I2C bus and the BMP180.
 *   - h_sample(): reads the sensor and saves the altitude in the global state.
 *   - bmp180_task(): FreeRTOS task that reads the barometer periodically.
 *   - height_init(): starts the I2C bus and the task.
 *   - height_test(): checks that the module started correctly.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ros_coordinator.h"


#include "state.h"
#include "height.h"

static const char *TAG = "h_task";

static geometry_msgs__msg__PoseStamped latest_data;

static portMUX_TYPE data_mux = portMUX_INITIALIZER_UNLOCKED;

#define H_TASK_PERIOD_MS 10

#define SEA_LEVEL_PA 101325.0f   // reference pressure for the altitude

static bool is_init = false;

#ifndef CONFIG_SIMULATION_ON
    static bmp180_t bmp;

    static esp_err_t i2c_master_init(void) {
        esp_err_t ret;

        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ,
        };

        ret = i2c_param_config(I2C_MASTER_NUM, &conf);
        // If everything is ok, continue
        if (ret != ESP_OK) {
            return ret;
        }

        // the IMU (same bus) can have installed the driver before.
        // i2c_driver_install() returns ESP_FAIL in that case.
        ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, ESP_INTR_FLAG_DEFAULT);
        if (ret != ESP_OK && ret != ESP_FAIL) {
            return ret;
        }

        ret = bmp180_init(&bmp, I2C_MASTER_NUM, BMP180_MODE_STANDARD);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "No se pudo inicializar el BMP180: %s", esp_err_to_name(ret));
            return ret;
        }
        return ret;
    }
#endif

static void h_sample() {
    #ifndef CONFIG_SIMULATION_ON
        int32_t pressure_pa;

        // bmp180_read_pressure() already reads the temperature
        if (bmp180_read_pressure(&bmp, &pressure_pa) != ESP_OK) {
            // ESP_LOGW(TAG, "Fallo leyendo el BMP180");
            return; // se conserva el ultimo dato valido, no se pisa con basura
        }

        float alt = bmp180_pressure_to_altitude(pressure_pa, SEA_LEVEL_PA);
        int64_t t_ns = esp_timer_get_time() * 1000; // Micro -> Nanosec

        set_h(alt);
        set_time_height(t_ns);

        // Keep get_height_data() up to date
        portENTER_CRITICAL(&data_mux);
        latest_data.pose.position.z      = alt;
        latest_data.header.stamp.sec     = (int32_t)(t_ns / 1000000000LL);
        latest_data.header.stamp.nanosec = (uint32_t)(t_ns % 1000000000LL);
        portEXIT_CRITICAL(&data_mux);
    #endif
}


static void bmp180_task(void *arg) {
    (void)arg;

    int log_counter = 0;

    while (1) {
        h_sample();

        if (++log_counter >= 500) {
            log_counter = 0;
            UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
            // // ESP_LOGI(TAG, "stack libre (min historico): %u bytes",
            //          (unsigned)(free_words * sizeof(StackType_t)));
        }

        vTaskDelay(pdMS_TO_TICKS(H_TASK_PERIOD_MS));
    }
}


void height_init(void) {
    if (is_init) {
        return;
    }

    esp_err_t ret;

    #ifndef CONFIG_SIMULATION_ON
        if (i2c_master_init() != ESP_OK) {
            ESP_LOGE(TAG, "Height not started");
            return;
        }
    #endif

    xTaskCreate(bmp180_task, "bmp180_task", CONFIG_HEIGHT_TASK_STACK, NULL, CONFIG_HEIGHT_TASK_PRIO, NULL);

    is_init = true;
}

bool height_test(void) {
    return is_init;
}
