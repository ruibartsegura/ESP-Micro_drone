/**
 * Made by Rui B.S.
 * Date: 24/09/226
 * email: rui.bartolome@gmail.com
 */

#include <stdbool.h>

#include "odometry_estimator.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static bool is_init = false;

void odom_estimator_init(void) {
    if (is_init) {
        return;
    }
    

    is_init = true;
}

bool odom_estimator_test(void) {
    if (!is_init) {
        return false;
    }

    return true;
}