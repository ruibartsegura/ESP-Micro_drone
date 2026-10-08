/**
 * Made by Rui B.S.
 * Date: 19/07/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Public interface, pins and PWM configuration of the motor driver.
 *
 * Functions:
 *   - arm_motors() / disarm_motors(): allows or blocks the motors.
 *   - motors_init(): configures the PWM timer and channels.
 *   - motors_test(): spins the motors to check them.
 *   - set_motor_speed(): sets the power of the 4 motors.
 *   - motors_stop_all(): stops all the motors.
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <stdbool.h>
#include <stdint.h>

#include "sdkconfig.h"


// PIN configuration
#define motor_1 CONFIG_MOTOR01_PIN
#define motor_2 CONFIG_MOTOR02_PIN
#define motor_3 CONFIG_MOTOR03_PIN
#define motor_4 CONFIG_MOTOR04_PIN

// LEDC configuration
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT   // 0-1023
#define LEDC_FREQUENCY  20000               // 20 kHz


#define N_MOTORS 4

void arm_motors(void);

void disarm_motors(void);

void motors_init(void);
bool motors_test(void); // Check leds

// Power of each motor: 0..100 % on the real drone, 0..MAX_POWER in simulation.
// The values are limited, and NaN or negative values become 0.
void set_motor_speed(double power[N_MOTORS]);

void motors_stop_all(void);

#endif // MOTORS_H
