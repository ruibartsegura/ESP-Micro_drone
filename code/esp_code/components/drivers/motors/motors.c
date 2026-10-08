/**
 * Made by Rui B.S.
 * Date: 19/07/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Driver for the 4 motors of the drone. On the real drone it drives the
 *   motors with PWM (LEDC, 20 kHz). In simulation mode it publishes the
 *   motor speeds to ROS 2 instead. The motors only move when the drone is
 *   armed, and the power is always limited to the maximum value.
 *
 * Functions:
 *   - arm_motors() / disarm_motors(): allows or blocks the motors.
 *   - set_motor_speed(): sets the power of the 4 motors.
 *   - motors_stop_all(): stops all the motors.
 *   - motors_init(): configures the PWM timer and channels.
 *   - motors_test(): spins the motors to check them.
 */

#include <stdbool.h>

#include "motors.h"
#include "ros_coordinator.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static const char *TAG  = "MOTOR";

static bool drone_armed = false;

static bool is_init     = false;
static bool is_testing  = false;

// GPIO pin and LEDC channel are just used with real drone, no simulation
#ifdef CONFIG_SIMULATION_ON
    #define TEST_POWER  2390
    #define MAX_POWER   4000
#else
    #define MAX_POWER   100
    #define TEST_POWER  100

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

static double limit_power(double power, double max) {
    if (!(power > 0.0)) {   // also true for NaN
        return 0.0;
    }
    if (power > max) {
        return max;
    }
    return power;
}

static void write_motors(const double power[N_MOTORS]) {
    #ifdef CONFIG_SIMULATION_ON
        pub_motor_speed(power);
    #else
        const uint32_t max_duty = (1u << LEDC_DUTY_RES) - 1;
        for (int x = 0; x < N_MOTORS; x++) {
            uint32_t duty = (uint32_t)(power[x] * max_duty / 100.0);
            ledc_set_duty(LEDC_MODE, motor_channel[x], duty);
            ledc_update_duty(LEDC_MODE, motor_channel[x]);
        }
    #endif
}

void arm_motors(void) {
    drone_armed = true;
}

void disarm_motors(void) {
    drone_armed = false;
}

void set_motor_speed(double power[N_MOTORS]) {
    if (is_testing) return;

    for (int x = 0; x < N_MOTORS; x++) {
        // When drone is disarmed, the motors doesn't work
        power[x] = drone_armed ? limit_power(power[x], MAX_POWER) : 0.0;
    }

    write_motors(power);
}

void motors_stop_all(void) {
    const double power[N_MOTORS] = {0};
    write_motors(power);
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
    double power[N_MOTORS] = {0};
    
    for (int x = 0; x < N_MOTORS; x++) {
        ESP_LOGI(TAG, "Motor %d", x);
        for (int vel = 0; vel <= TEST_POWER/2; vel = vel + 50) {
            power[x] = vel;
            write_motors(power);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        ESP_LOGI(TAG, "MAX POWER");
        for (int vel = TEST_POWER/2; vel >= 0; vel = vel - 100) {
            power[x] = vel;
            write_motors(power);
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
