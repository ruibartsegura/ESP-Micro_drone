#include "system.h"

#include <stdbool.h>
#include <stdint.h>
#include <geometry_msgs/msg/twist.h>


// Free RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Drivers
#include "led.h"
#include "imu.h"
#include "attitude_controller.h"

// Microros
#include "ros_coordinator.h"

// How much error it's admitted in the altitude
#define ALTITUDE_ERROR 5

// Target of velocities + height
ATTITUDE_TARGET att_target;

// Hover velocity = 0;
geometry_msgs__msg__Twist hover_vel = {0};



int state = INIT;
static bool is_init = false;

void system_init(void) {
    if (is_init) {
        return;
    }

    led_init(); // Primero en init

    vTaskDelay(pdMS_TO_TICKS(500));

    imu_init();

    vTaskDelay(pdMS_TO_TICKS(500));
    led_on(LED_BLUE);

    
    ros_init(); // último en init

    vTaskDelay(pdMS_TO_TICKS(1000));

    is_init = true;
}

bool system_test(void) {
    bool test = is_init;

    test &= led_test();
    test &= imu_test();
    test &= ros_test();

    return test;
}


// Change the state-machine state 
void change_state(int new_state) {
    state = new_state;
}

// Return the state-machine state 
int get_state() {
    return state;
}

// Return the desired attitude for the drone
ATTITUDE_TARGET get_attitude() {
    return att_target;
}

// TODO: Ver código pq tal vez hay que moverlo a attitude_controller
// Check if the take_off has reached the desired altitude
bool check_takeOff_2_hov(float hov_h) {
    float h = 0; //get_h();

    float above_error, below_error;
    above_error = hov_h + hov_h * (ALTITUDE_ERROR / 100.0f);
    below_error = hov_h - hov_h * (ALTITUDE_ERROR / 100.0f);

    if(h > below_error && h < above_error) {
        return true;
    } else {
        return false;
    }
}

void state_machine(void) {

    switch (state) {
        case INIT:
            system_init();
            
            change_state(CHECKING);
            
            break;

        case CHECKING:
            // While this state the params can be updated
            if (system_test()) {
                change_state(ARMING);
                led_on(LED_GREEN);
            } else {
                change_state(ERROR);
            }
            break;

        case ARMING:
            // start_engine();

            if(get_take_off_ready()) {
                for(int t=0; t < 3; t++) {
                    led_on(LED_BLUE);
                    vTaskDelay(pdMS_TO_TICKS(150));
                }


                change_state(TAKING_OFF);
            } else {
                // Check if take_off service has arrive each 150ms
                vTaskDelay(pdMS_TO_TICKS(150));
            }
            break;

        case TAKING_OFF:
            att_target.cmd_vel = hover_vel;
            att_target.h = get_take_off_alt();

            if (check_takeOff_2_hov(att_target.h)) {
                change_state(HOVERING);
            }
            break;

        case HOVERING:
            att_target.cmd_vel = hover_vel;
            att_target.h = get_take_off_alt();


            // if (new_vel()) {
            //     change_state(EXTERNAL_CONTROL);
            // }

            // if (get_land()) {
            //     change_state(LANDING);
            // }
            break;

        case EXTERNAL_CONTROL:
            att_target.cmd_vel = get_cmd_vel(); // TODO: revisar caducidad del cmd_vel
            att_target.h = get_take_off_alt();

            // if (!get_new_pos()) {
            //     change_state(HOVERING);
            // }

            // if (get_land()) {
            //     change_state(LANDING);
            // }
            break;

        case LANDING:
            att_target.cmd_vel = hover_vel;
            att_target.h = 0;

            // if (get_land()) {
            //     change_state(LANDING);
            // }
            break;

        case DISARMING:
            // shut_down_engine();
            break;

        case ERROR:
            break;

    }
}

// El delay es importante: la S2-mini es single-core
// (CONFIG_FREERTOS_UNICORE=y). Sin ceder CPU en cada vuelta, estados
// sin delay interno (CHECKING, HOVERING, ERROR...) acapararían el
// único core, matarían de hambre a la tarea IDLE y dispararían el
// Task Watchdog Timer (reset inesperado del ESP32).
#define SYSTEM_TASK_PERIOD_MS 20

static void system_task(void *arg) {
    (void)arg;
    int log_counter = 0;

    while (1) {
        state_machine();

        if (++log_counter >= 250) {
            log_counter = 0;
            UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI("system_task", "stack libre (min historico): %u bytes",
                     (unsigned)(free_words * sizeof(StackType_t)));
        }

        vTaskDelay(pdMS_TO_TICKS(SYSTEM_TASK_PERIOD_MS));
    }
}

void system_start(void) {
    xTaskCreate(system_task, "system_task", CONFIG_SYSTEM_TASK_STACK, NULL, CONFIG_SYSTEM_TASK_PRIO, NULL);
}

// TODO
//      Implementar get_take_off() en ros_coordinator
// 
//      Implementar go_to(x, y, h) en X
//
//      Implementar params en configuration, hacer get/set para modificarlo con ros_coordinator
//
//