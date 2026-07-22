/**
 * Made by Rui B.S.
 * Date: 19/07/2026
 * email: rui.bartolome@gmail.com
 */

#include <stdbool.h>

#include "motors.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static const char *TAG = "MOTOR";

static bool is_init = false;

static const gpio_num_t motor_gpio[N_MOTORS] = {
    motor_1,
    motor_2,
    motor_3,
    motor_4,
};

static const ledc_channel_t motor_channel[N_MOTORS] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3
};


void motor_set_speed(uint8_t motor_id, uint8_t spd_percent) {
    if (motor_id >= N_MOTORS) {
        return;
    }

    if (spd_percent > 100) {
        spd_percent = 100;
    }

    uint32_t duty = (spd_percent * ((1 << LEDC_DUTY_RES) - 1)) / 100;
    ledc_set_duty(LEDC_MODE, motor_channel[motor_id], duty);
    ledc_update_duty(LEDC_MODE, motor_channel[motor_id]);
}

void motors_stop_all(void) {
    for (int i = 0; i < N_MOTORS; i++) {
        ledc_set_duty(LEDC_MODE, motor_channel[i], 0);
        ledc_update_duty(LEDC_MODE, motor_channel[i]);
    }
}

void motors_init(void) {
    if (is_init) {
        return;
    }
    
    // One timer shared for the 4 motors
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = LEDC_FREQUENCY,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);
    
    // One chanel for motor, all pointing same timer
    for (int i = 0; i < N_MOTORS; i++) {
        ledc_channel_config_t channel_conf = {
            .gpio_num   = motor_gpio[i],
            .speed_mode = LEDC_MODE,
            .channel    = motor_channel[i],
            .timer_sel  = LEDC_TIMER,
            .duty       = 0,     // arranca apagado
            .hpoint     = 0
        };
        ledc_channel_config(&channel_conf);
    }
    
    is_init = true;
}

bool motors_test(void) {
    if (!is_init) {
        return false;
    }
    
    ESP_LOGI(TAG, "Empieza test");
    
    for (int x = 0; x < N_MOTORS; x++) {
        for (int vel = 0; vel <= 100; vel = vel + 10) {
            motor_set_speed(x, vel);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        for (int vel = 100; vel >= 0; vel = vel - 10) {
            motor_set_speed(x, vel);
            vTaskDelay(pdMS_TO_TICKS(150));
        }

    }
    
    ESP_LOGI(TAG, "Termina test");
    return true;
}
