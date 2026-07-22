#ifndef MOTORS_H
#define MOTORS_H

#include <stdbool.h>
#include <stdint.h>

#include "sdkconfig.h"


// PIN configuration
#define motor_1 CONFIG_LED_PIN_ESP
#define motor_2 CONFIG_LED_PIN_RED
#define motor_3 CONFIG_LED_PIN_GREEN
#define motor_4 CONFIG_LED_PIN_BLUE

// LEDC configuration
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT   // 0-1023
#define LEDC_FREQUENCY  20000               // 20 kHz


#define N_MOTORS 4

void motors_init(void);
bool motors_test(void); // Check leds

void motor_set_speed(uint8_t motor_id, uint8_t spd_percent);

void motors_stop_all(void);

#endif // MOTORS_H
