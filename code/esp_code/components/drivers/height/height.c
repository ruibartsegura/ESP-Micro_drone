/*
 * main_bmp180_example.c
 *
 * Ejemplo de uso del driver bmp180.c/.h en un proyecto ESP-IDF para
 * ESP32-S2. Configura el bus I2C (compartido con el resto del GY-87:
 * MPU6050 en 0x68 y HMC5883L en 0x1E/0x0D) y lanza una tarea que lee
 * temperatura y presion cada segundo.
 *
 * Ajusta GPIO_SDA / GPIO_SCL a tu cableado real.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"


#include "height.h"

static const char *TAG = "h_task";

static geometry_msgs__msg__PoseStamped latest_data;

static portMUX_TYPE data_mux = portMUX_INITIALIZER_UNLOCKED;

#define H_TASK_PERIOD_MS 10

static bool is_init = false;

#ifndef CONFIG_SIMULATION_ON
    static bmp180_t bmp;

    static esp_err_t i2c_master_init(void) {
        esp_err_t ret;

        i2c_config_t conf = {
            .mode = I2C_MASTER_NUM,
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

        ret =  i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
        if (ret != ESP_OK) {
            return ret;
        }

        ret = bmp180_init(&bmp, I2C_MASTER_NUM, BMP180_MODE_STANDARD);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "No se pudo inicializar el BMP180: %s", esp_err_to_name(ret));
            vTaskDelete(NULL);
            return ret;
        }
        return ret;
    }
#endif

esp_err_t get_height_data(geometry_msgs__msg__PoseStamped *data) {
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

static void h_sample() {
    geometry_msgs__msg__PoseStamped sample;

    #ifdef CONFIG_SIMULATION_ON
        sample = get_height_sim();
    #else
        float temp_c;
        int32_t pressure_pa;

        if (bmp180_read_temperature(&bmp, &temp_c) == ESP_OK &&
            bmp180_read_pressure(&bmp, &pressure_pa) == ESP_OK) {

            sample.pose.position.z = bmp180_pressure_to_altitude(pressure_pa, 101325.0f);
            sample.header.stamp.nanosec = esp_timer_get_time() * 1000; // Micro -> Nanosec

            ESP_LOGI(TAG, "Temp: %.1f C | Presion: %ld Pa | Altitud aprox: %.2f m",
                        temp_c, (long)pressure_pa, sample.pose.position.z);
        } else {
            ESP_LOGW(TAG, "Fallo leyendo el BMP180");
            return; // se conserva el ultimo dato valido, no se pisa con basura
        }
    #endif

    portENTER_CRITICAL(&data_mux);
    latest_data = sample;
    portEXIT_CRITICAL(&data_mux);
}


static void bmp180_task(void *arg)
{
    (void)arg;

    int log_counter = 0;

    while (1) {
        h_sample();

        if (++log_counter >= 500) {
            log_counter = 0;
            UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI(TAG, "stack libre (min historico): %u bytes",
                     (unsigned)(free_words * sizeof(StackType_t)));
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
        ESP_ERROR_CHECK(i2c_master_init());
    #endif

    xTaskCreate(bmp180_task, "bmp180_task", 4096, NULL, 5, NULL);

    is_init = true;
}

bool height_test(void) {
    return is_init;
}
