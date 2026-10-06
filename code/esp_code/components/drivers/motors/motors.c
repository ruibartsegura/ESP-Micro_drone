/**
 * Made by Rui B.S.
 * Date: 19/07/2026
 * email: rui.bartolome@gmail.com
 */

#include <stdbool.h>

#include "motors.h"
#include "ros_coordinator.h"
#include "ros_coordinator.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static const char *TAG = "MOTOR";

static bool drone_armed = false;

static bool is_init = false;
static bool is_testing = false;

// GPIO pin and LEDC channel are just used with real drone, no simulation
#ifdef CONFIG_SIMULATION_ON
    #define TEST_VAL 2387
#else
    #define TEST_VAL 100
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
#endif

void arm_motors() {
    drone_armed = true;
}

void disarm_motors() {
    drone_armed = false;
}

void set_motor_speed(double power[N_MOTORS]) {
    if (is_testing) return;
    // When drone is disarmed, the motors doesn't work
    if (!drone_armed) {
        for (int x = 0; x < N_MOTORS; x++) {
            power[x] = 0;
        }
    }

    // With simulation pass the motor speed to ROS2 pub
    #ifdef CONFIG_SIMULATION_ON
        pub_motor_speed(power);

    #else
        for (int x = 0; x < N_MOTORS; x++) {
            uint8_t spd_percent[N_MOTORS] = power; // TODO Cambiar necesito modelo real

            // Clamp motor spdeed
            if (spd_percent[x] > 100) {
                spd_percent[x] = 100;
            }
            uint32_t duty = (spd_percent[x] * ((1 << LEDC_DUTY_RES) - 1)) / 100;
            ledc_set_duty(LEDC_MODE, motor_channel[x], duty);
            ledc_update_duty(LEDC_MODE, motor_channel[x]);
        }
    #endif
}

void motors_stop_all(void) {
    // With simulation pass the motor speed to ROS2 pub
    #ifdef CONFIG_SIMULATION_ON
        double power[N_MOTORS] = {0};
        pub_motor_speed(power);

    #else
        for (int i = 0; i < N_MOTORS; i++) {
            ledc_set_duty(LEDC_MODE, motor_channel[i], 0);
            ledc_update_duty(LEDC_MODE, motor_channel[i]);
        }
    #endif

}

void motors_init(void) {
    if (is_init) {
        return;
    }
    
    // With simulation no need to declare the timers for LEDC...
    #ifndef CONFIG_SIMULATION_ON
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
    #endif

    is_init = true;
}

bool motors_test(void) {
    if (!is_init) {
        return false;
    }
    is_testing = true;
    
    ESP_LOGI(TAG, "Empieza test");
    double power[N_MOTORS];
    
    for (int x = 0; x < N_MOTORS; x++) {
        ESP_LOGI(TAG, "Motor %d", x);
        for (int vel = 0; vel <= TEST_VAL/2; vel = vel + 50) {
            power[x] = vel;
            pub_motor_speed(power);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        ESP_LOGI(TAG, "MAX POWER");
        for (int vel = TEST_VAL/2; vel >= 0; vel = vel - 100) {
            power[x] = vel;
            pub_motor_speed(power);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        power[x] = 0;
        ESP_LOGI(TAG, "FINISH MOTOR X");
    }
    
    // Make sure the motors are stopped before finish the test
    motors_stop_all();
    vTaskDelay(pdMS_TO_TICKS(150));

    
    ESP_LOGI(TAG, "Termina test");
    is_testing = false;
    return true;
}
