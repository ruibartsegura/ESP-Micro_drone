/**
 * Made by Rui B.S.
 * Date: 22/07/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Core of the drone. It initialises every module (state, LEDs, sensors,
 *   motors, attitude controller and micro-ROS) and runs the state machine
 *   in its own FreeRTOS task every 20 ms:
 *   INIT -> CHECKING -> ARMING -> TAKING_OFF -> HOVERING <-> EXTERNAL_CONTROL
 *   -> LANDING -> DISARMING. ERROR is a final state.
 *   In simulation mode the IMU and barometer drivers are not started.
 *
 * Functions:
 *   - system_init(): initialises all the modules in order (micro-ROS last).
 *   - system_test(): checks that all the modules started correctly.
 *   - change_state(): changes the state machine state and saves it in the global state.
 *   - get_attitude(): returns the target attitude (cmd_vel + height).
 *   - check_takeOff_2_hov(): checks if the take-off height has been reached.
 *   - state_machine(): runs one step of the state machine.
 *   - system_task(): FreeRTOS task that calls state_machine() periodically.
 *   - system_start(): creates the system task.
 */

#include "system.h"

#include <stdbool.h>
#include <stdint.h>
#include <geometry_msgs/msg/twist.h>


// Free RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Global_data
#include "state.h"

// Drivers
#include "led.h"
#include "imu.h"
#include "height.h"
#include "motors.h"

#include "attitude_controller.h"
#include "tuning.h"

// Microros
#include "ros_coordinator.h"

#define SYSTEM_TASK_PERIOD_MS 20

// How much error it's admitted in the altitude
#define ALTITUDE_ERROR 5

static const char *TAG = "SYSTEM";

// Target of velocities + height
ATTITUDE_TARGET att_target;

// Hover velocity = 0;
geometry_msgs__msg__Twist hover_vel = {0};


sm_states_t state = INIT;
static bool is_init = false;

void system_init(void) {
    if (is_init) {
        return;
    }

    state_init();
    vTaskDelay(pdMS_TO_TICKS(500));

    led_init();
    vTaskDelay(pdMS_TO_TICKS(500));

    #ifndef CONFIG_SIMULATION_ON

        imu_init();
        vTaskDelay(pdMS_TO_TICKS(500));

        height_init();
        vTaskDelay(pdMS_TO_TICKS(500));
    #endif

    motors_init();
    vTaskDelay(pdMS_TO_TICKS(500));

    init_attitude_controller();
    vTaskDelay(pdMS_TO_TICKS(500));

    #ifdef CONFIG_GAINS_TUNE_ON
        init_tuning(); // change the gains from the serial console
    #endif

    ros_init(); // last init
    vTaskDelay(pdMS_TO_TICKS(1000));

    is_init = true;
}

bool system_test(void) {
    bool test = is_init;

    test &= led_test();

    #ifndef CONFIG_SIMULATION_ON
        test &= imu_test();
        test &= height_test();
    #endif

    //test &= motors_test();

    test &= ros_test();

    ESP_LOGI(TAG, "Fin test result %d", test);
    return test;
}


// Change the state-machine state
void change_state(int new_state) {
    state = new_state;
    set_sm_state(state);
}

// Return the desired attitude for the drone
ATTITUDE_TARGET get_attitude() {
    return att_target;
}


static bool blink_led = true;

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
            if(get_take_off_ready()) {
                arm_motors();

                change_state(TAKING_OFF);
            } else {
                // Check if take_off service has arrive each 150ms
                vTaskDelay(pdMS_TO_TICKS(150));
            }
            break;

        case TAKING_OFF:
            att_target.cmd_vel = hover_vel;
            att_target.h = get_take_off_alt();

            if (check_h_reached()) {
                led_on(LED_BLUE);
                change_state(HOVERING);
            } else {
                if (blink_led) {
                    led_on(LED_BLUE);
                    blink_led = false;
                    vTaskDelay(pdMS_TO_TICKS(150));
                } else {
                    led_off(LED_BLUE);
                    blink_led = true;
                    vTaskDelay(pdMS_TO_TICKS(150));
                }
            }
            break;

        case HOVERING:
            att_target.cmd_vel = hover_vel;
            att_target.h = get_take_off_alt();


            if (get_new_vel()) {
                change_state(EXTERNAL_CONTROL);
            }

            if (get_landing_ready()) {
                led_on(LED_GREEN);
                led_off(LED_BLUE);

                change_state(LANDING);
            }
            break;

        case EXTERNAL_CONTROL:
            att_target.cmd_vel = get_cmd_vel();
            att_target.h = get_take_off_alt();

            if (get_landing_ready()) {
                led_on(LED_GREEN);
                led_off(LED_BLUE);

                change_state(LANDING);
            } else {
                if (blink_led) {
                    led_on(LED_GREEN);
                    blink_led = false;
                    vTaskDelay(pdMS_TO_TICKS(150));
                } else {
                    led_off(LED_GREEN);
                    blink_led = true;
                    vTaskDelay(pdMS_TO_TICKS(150));
                }
            }
            break;

        case LANDING:
            att_target.cmd_vel = hover_vel;
            att_target.h = 0;

            if (check_h_reached()) {
                led_on(LED_RED);
                led_on(LED_GREEN);
                led_on(LED_BLUE);

                change_state(DISARMING);
            } else {
                 if (blink_led) {
                    led_on(LED_RED);
                    blink_led = false;
                    vTaskDelay(pdMS_TO_TICKS(150));
                } else {
                    led_off(LED_RED);
                    blink_led = true;
                    vTaskDelay(pdMS_TO_TICKS(150));
                }
            }
            break;

        case DISARMING:
            motors_stop_all();
            disarm_motors();

            led_on(LED_GREEN);
            led_off(LED_RED);
            led_off(LED_BLUE);

            change_state(ARMING);

            break;

        case ERROR:
            break;

    }
}


static void system_task(void *arg) {
    (void)arg;
    int log_counter = 0;

    while (1) {
        state_machine();

        if (++log_counter >= 250) {
            log_counter = 0;
            UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
            // ESP_LOGI("system_task", "stack libre (min historico): %u bytes",
                    //  (unsigned)(free_words * sizeof(StackType_t)));
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
//      Implementar params en configuration, hacer get/set para modificarlo con ros_coordinator
//
//      Hacer cambios de estados...
