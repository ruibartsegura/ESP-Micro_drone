/**
 * Made by Rui B.S.
 * Date: 24/09/226
 * email: rui.bartolome@gmail.com
 */

#include <stdbool.h>

#include "led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

void led_init(void) {
    if (is_init) {
        return;
    }
    

    is_init = true;
}

bool led_test(void) {
    if (!is_init) {
        return false;
    }

    return true;
}