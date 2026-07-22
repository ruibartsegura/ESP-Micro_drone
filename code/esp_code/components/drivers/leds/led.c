/**
 * Made by Rui B.S.
 * Date: 23/05/226
 * email: rui.bartolome@gmail.com
 */

#include <stdbool.h>

#include "led.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static const char *TAG = "LED";

static bool is_init = false;

static unsigned int led_pin[] = {
    [LED_ESP]   = ESP_LED_PIN,
    [LED_RED]   = RED_LED_PIN,
    [LED_GREEN] = GREEN_LED_PIN,
    [LED_BLUE]  = BLUE_LED_PIN,
};
static int led_status[] = {
    [LED_ESP]   = ESP_LED_STATUS,
    [LED_RED]   = RED_LED_STATUS,
    [LED_GREEN] = GREEN_LED_STATUS,
    [LED_BLUE]  = BLUE_LED_STATUS,
};

void led_init(void) {
    if (is_init) {
        return;
    }
    
    for (int x = 0; x < N_LEDS; x++) {
        gpio_config_t io_conf = {
            //bit mask of the pins that you want to set,e.g.GPIO18/19
            .pin_bit_mask = (1ULL << led_pin[x]),
            //disable pull-down mode
            .pull_down_en = 0,
            //disable pull-up mode
            .pull_up_en = 0,
            //set as output mode
            .mode = GPIO_MODE_OUTPUT,
        };

        gpio_config(&io_conf);

        gpio_set_level(led_pin[x], 0);
    }

    is_init = true;
}

bool led_test(void) {
    if (!is_init) {
        return false;
    }

    ESP_LOGI(TAG, "Empieza test");
    
    for (int x = 0; x < N_LEDS; x++) {
        gpio_set_level(led_pin[x], 1);
        vTaskDelay(pdMS_TO_TICKS(250));
        
        gpio_set_level(led_pin[x], 0);
        vTaskDelay(pdMS_TO_TICKS(250));

        gpio_set_level(led_pin[x], 1);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    
    ESP_LOGI(TAG, "Termina test");
    all_off();
    return true;
}

void led_on(led_t led) {
    if (led_status[led] == LED_OFF) {
        gpio_set_level(led_pin[led], LED_ON);
        led_status[led] = LED_ON;
    }
}

void led_off(led_t led) {
    if (led_status[led] == LED_ON) {
        gpio_set_level(led_pin[led], LED_OFF);
        led_status[led] = LED_OFF;
    }
}

void all_on(void) {
    for (int x = 0; x < N_LEDS; x++) {
        gpio_set_level(led_pin[x], LED_ON);
        led_status[x] = LED_ON;
    }
}

void all_off(void) {
    for (int x = 0; x < N_LEDS; x++) {
        gpio_set_level(led_pin[x], LED_OFF);
        led_status[x] = LED_OFF;
    }
}
