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
 *   - limit_power(): limits the power of one motor to [0, max].
 *   - write_motors(): writes the power to the PWM (or publishes it in simulation).
 *   - arm_motors() / disarm_motors(): allows or blocks the motors.
 *   - set_motor_speed(): sets the power of the 4 motors.
 *   - motors_stop_all(): stops all the motors.
 *   - init_motors(): configures the PWM timer and channels.
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

static const char *TAG = "MOTOR";

static volatile bool drone_armed = false;

static bool is_init = false;
static volatile bool is_testing = false;

// GPIO pin and LEDC channel are just used with real drone, no simulation
#ifdef CONFIG_SIMULATION_ON
    #define TEST_POWER  2390
    #define MAX_POWER   3000
#else
    #define TEST_POWER  100
    #define MAX_POWER   100
    #define VEL_MAX_RADS    1500.0
    #define VEL_HOVER_RADS   600.0
    #define POWER_HOVER_PCT     45.0
    #define POWER_MIN_PCT        0.0

    static const gpio_num_t motor_gpio[N_MOTORS] = {
        MOTOR_1,
        MOTOR_2,
        MOTOR_3,
        MOTOR_4,
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

#ifndef CONFIG_SIMULATION_ON
static double vel_2_percent(double vel) {
    if (!(vel > 0.0)) return 0.0;
    if (vel >= VEL_MAX_RADS) return 100.0;

    if (vel <= VEL_HOVER_RADS) {
        return POWER_MIN_PCT
             + (POWER_HOVER_PCT - POWER_MIN_PCT) * vel / VEL_HOVER_RADS;
    }
    return POWER_HOVER_PCT
         + (100.0 - POWER_HOVER_PCT)
           * (vel - VEL_HOVER_RADS) / (VEL_MAX_RADS - VEL_HOVER_RADS);
}
#endif

static void write_motors(const double power[N_MOTORS]) {
    #ifdef CONFIG_SIMULATION_ON
        pub_motor_speed(power);
    #else
        const uint32_t max_duty = (1u << LEDC_DUTY_RES) - 1;
        for (int x = 0; x < N_MOTORS; x++) {
            double percent = vel_2_percent(power[x]);   // <-- aquí
            uint32_t duty = (uint32_t)(percent * max_duty / 100.0);
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
        if (drone_armed) {
            power[x] = limit_power(power[x], MAX_POWER);
        } else {
            power[x] = 0;
        }
    }

    write_motors(power);
}

void motors_stop_all(void) {
    const double power[N_MOTORS] = {0};
    write_motors(power);
}

void init_motors(void) {
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
                .duty       = 0, // arranca apagado
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

    double power[N_MOTORS] = {0};

    for (int x = 0; x < N_MOTORS; x++) {
        for (int vel = 0; vel <= TEST_POWER/2; vel = vel + 50) {
            power[x] = vel;
            write_motors(power);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        for (int vel = TEST_POWER/2; vel >= 0; vel = vel - 100) {
            power[x] = vel;
            write_motors(power);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        power[x] = 0;
    }

    // Make sure the motors are stopped before finish the test
    motors_stop_all();
    vTaskDelay(pdMS_TO_TICKS(150));


    is_testing = false;
    return true;
}
