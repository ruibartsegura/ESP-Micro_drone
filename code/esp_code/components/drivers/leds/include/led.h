/**
 * Made by Rui B.S.
 * Date: 23/05/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Public interface, pins and LED ids of the LED driver.
 *
 * Functions:
 *   - led_init(): configures the LED GPIOs.
 *   - led_test(): turns all the LEDs on and off to check them.
 *   - led_on() / led_off(): turns one LED on or off.
 *   - all_on() / all_off(): turns all the LEDs on or off.
 */

#ifndef LED_H
#define LED_H

#include <stdbool.h>

#include "sdkconfig.h"


// PIN configuration
#define ESP_LED_PIN CONFIG_LED_PIN_ESP
#define RED_LED_PIN CONFIG_LED_PIN_RED
#define GREEN_LED_PIN CONFIG_LED_PIN_GREEN
#define BLUE_LED_PIN CONFIG_LED_PIN_BLUE

// LED status
#define LED_OFF 0
#define LED_ON 1

#define ESP_LED_STATUS LED_OFF
#define RED_LED_STATUS LED_OFF
#define GREEN_LED_STATUS LED_OFF
#define BLUE_LED_STATUS LED_OFF

// LEDS
#define N_LEDS 4

typedef enum {LED_ESP = 0, LED_BLUE, LED_RED, LED_GREEN} led_t;


void led_init(void);
bool led_test(void); // Check leds

void led_on(led_t led);
void led_off(led_t led);
void all_on(void);
void all_off(void);

#endif // LED_H
