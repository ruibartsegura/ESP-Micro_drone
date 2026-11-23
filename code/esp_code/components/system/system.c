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
 *   If the IMU or the barometer stop sending data while flying, it goes to
 *   EMERGENCY_LANDING and then to ERROR with the motors stopped. The same if
 *   the link with the micro-ROS agent is lost (real drone): it lands with
 *   its sensors, without ROS, and then goes to ERROR.
 *   In simulation mode the IMU and barometer drivers are not started.
 *
 * Functions:
 *   - system_init(): initialises all the modules in order (micro-ROS last).
 *   - system_test(): checks that all the modules started correctly.
 *   - change_state(): changes the state machine state and saves it in the global state.
 *   - get_attitude(): returns the target attitude (cmd_vel + height).
 *   - get_barometer_failing() / get_imu_failing(): failure of the emergency.
 *   - check_sensors(): looks for sensors that stopped sending data and for
 *     a lost link (real drone).
 *   - start_emergency(): starts the emergency landing.
 *   - finish_emergency(): stops the motors and goes to ERROR.
 *   - imu_cut_step(): power ramp to 0 when the IMU fails.
 *   - link_land_step(): landing with the sensors when the link is lost.
 *   - blink(): blinks a LED without stopping the state machine.
 *   - state_machine(): runs one step of the state machine.
 *   - system_task(): FreeRTOS task that calls state_machine() periodically.
 *   - system_start(): creates the system task.
 */

#include "system.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <geometry_msgs/msg/twist.h>


// Free RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

// Global_data
#include "state.h"

// Diagnostics
#include "cpu_stats.h"

// Drivers
#include "led.h"
#include "imu.h"
#include "height.h"
#include "motors.h"

// Flight control
#include "attitude_controller.h"
#include "odometry_estimator.h"
#include "tuning.h"

// Microros
#include "ros_coordinator.h"

// Target of velocities + height
static ATTITUDE_TARGET att_target;

// Hover velocity = 0;
static geometry_msgs__msg__Twist hover_vel = {0};

// Expire time for cmd vel msg
#define CMD_VEL_EXPIRE_TIME 500

// Max time without new sensor data before starting the failsafe
#define IMU_TIMEOUT_US (CONFIG_IMU_TIMEOUT * 1000LL)

#define BARO_TIMEOUT_US (CONFIG_BARO_TIMEOUT * 1000LL)

#define LINK_TIMEOUT_US (CONFIG_LINK_TIMEOUT * 1000LL)

// Types of failure. A lower value is worse: it replaces a higher one
typedef enum {
    SENSORS_OK   = 0,
    IMU_FAILURE  = 1,
    BARO_FAILURE = 2,
    LINK_FAILURE = 3  // Link with the micro-ROS agent lost (real drone)
} failure_type_t;


// Emergency landing
#define IMU_CUT_TIME_MS 500 // Power ramp to 0 without IMU (no stabilization)
#define EMRGY_ACC 0.5f // Expected descent acceleration without barometer (m/s^2)
#define EMRGY_MARGIN_S 1.0f // Extra time before stopping the motors without barometer
#define LINK_MIN_DESC_VEL 0.15f // Slowest descent expected without link (m/s)
#define LINK_MARGIN_S 3.0f // Extra time before stopping the motors without link


#define SYSTEM_TASK_PERIOD_MS 20
static const char *TAG = "SYSTEM";

static sm_states_t state = INIT;
static volatile failure_type_t f_type = SENSORS_OK;
static failure_type_t f_emergency = SENSORS_OK; // Failure of the current emergency

static int64_t t_emergency_us = 0; // Start of the emergency landing
static int64_t emergency_time_us = 0; // Duration of the landing without barometer
static int64_t t_imu_cut_us = -1; // Start of the power ramp (-1 = not started)
static float imu_cut_start = 0.0f; // Power at the start of the ramp

static bool is_init = false;

// Return the desired attitude for the drone
ATTITUDE_TARGET get_attitude() {
    return att_target;
}

bool get_barometer_failing(void) {
    if (f_type == BARO_FAILURE) {
        return true;
    } else {
        return false;
    }
}

bool get_imu_failing(void) {
    return f_type == IMU_FAILURE;
}


void system_init(void) {
    if (is_init) {
        return;
    }

    // Initialize the global state
    state_init();

    #ifdef CONFIG_CPU_STATS_ON
        init_cpu_stats(); // CPU / period log (only with CONFIG_CPU_STATS_ON)
    #endif

    vTaskDelay(pdMS_TO_TICKS(500));

    // Initialize led
    led_init();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Initialize the sensors if we are not in simulation mode.
    #ifndef CONFIG_SIMULATION_ON
        imu_init();
        vTaskDelay(pdMS_TO_TICKS(500));

        height_init();
        vTaskDelay(pdMS_TO_TICKS(500));
    #endif

    // Initialize the motors
    init_motors();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Initialize the attitude controller
    init_attitude_controller();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Initialize the odometry estimator
    init_odom_estimator();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // If gains tunning on initialize it.
    #ifdef CONFIG_GAINS_TUNE_ON
        init_tuning(); // change the gains from the serial console
        vTaskDelay(pdMS_TO_TICKS(500));
    #endif

    // Initialize external connection
    ros_init(); // last init
    vTaskDelay(pdMS_TO_TICKS(1000));

    is_init = true;
}

bool system_test(void) {
    bool test = is_init;

    // Visual test
    test &= led_test();

    // Test if the subscriber is initialize in the simulation
    #ifndef CONFIG_SIMULATION_ON
        test &= imu_test();
        test &= height_test();
    #endif

    // Visual test
    test &= motors_test();

    test &= odom_estimator_test();
    test &= attitude_controller_test();
    test &= ros_test();

    return test;
}


// Change the state-machine state
void change_state(sm_states_t new_state) {
    state = new_state;
    set_sm_state(state);
}

static void check_sensors(void) {
    int64_t now = esp_timer_get_time();
    int64_t t_imu, t_h;

    get_t_last_sample_imu(&t_imu);
    get_t_last_sample_h(&t_h);

    failure_type_t f = SENSORS_OK;
    if (now - t_imu > IMU_TIMEOUT_US) {
        f = IMU_FAILURE;
    } else if (now - t_h > BARO_TIMEOUT_US) {
        f = BARO_FAILURE;
    }
    #ifndef CONFIG_SIMULATION_ON
        // In simulation the sensors come from ROS: the IMU timeout is the link
        else if (now - get_t_last_link() > LINK_TIMEOUT_US) {
            f = LINK_FAILURE;
        }
    #endif

    // The failure stays until ERROR; only a worse one replaces it
    if (f != SENSORS_OK && (f_type == SENSORS_OK || f < f_type)) {
        if (f == LINK_FAILURE) {
            ESP_LOGW(TAG, "Link with the agent lost: landing");
        }
        f_type = f;
    }
}

static void start_emergency(void) {
    float h = get_h_est();
    if (h < 0.0f) {
        h = 0.0f;
    }

    t_emergency_us = esp_timer_get_time();
    if (f_type == LINK_FAILURE) {
        // Controlled landing: max time with the slowest descent + margin
        emergency_time_us = (int64_t)((h / LINK_MIN_DESC_VEL + LINK_MARGIN_S) * 1000000.0f);
        reset_h_reached();
    } else {
        // Time to fall from h with a constant acceleration + margin
        emergency_time_us = (int64_t)((sqrtf(2.0f * h / EMRGY_ACC) + EMRGY_MARGIN_S) * 1000000.0f);
    }

    f_emergency = f_type;
    change_state(EMERGENCY_LANDING);
}

static void finish_emergency(void) {
    // First block the motors, then stop them
    disarm_motors();
    motors_stop_all();
    change_state(ERROR);
}

// Without IMU there is no stabilization: same power in the 4 motors, down to 0
static void imu_cut_step(void) {
    int64_t now = esp_timer_get_time();
    if (t_imu_cut_us < 0) {
        t_imu_cut_us = now;
        imu_cut_start = get_last_motor_power();
    }

    float elapsed_ms = (now - t_imu_cut_us) / 1000.0f;
    if (elapsed_ms >= IMU_CUT_TIME_MS) {
        finish_emergency();
        return;
    }

    double power[N_MOTORS];
    for (int x = 0; x < N_MOTORS; x++) {
        power[x] = imu_cut_start * (1.0f - elapsed_ms / IMU_CUT_TIME_MS);
    }
    set_motor_speed(power);
}

// Without link the sensors are ok: same landing as LANDING (h = 0), but
// without ROS. If the ground is not detected in time, it stops anyway
static void link_land_step(void) {
    att_target.h = 0;

    if (check_h_reached()
        || esp_timer_get_time() - t_emergency_us >= emergency_time_us) {
        finish_emergency();
    }
}

// Variables to make the leds blink
static bool blink_led = true;
#define BLINK_PERIOD_MS 150
static int64_t t_blink_us = -1; // Time of the last change of the LED (-1 = never)

static void blink(led_t led) {
    int64_t now = esp_timer_get_time();
    if (t_blink_us >= 0 && now - t_blink_us < BLINK_PERIOD_MS * 1000LL) {
        return;
    }
    t_blink_us = now;

    if (blink_led) {
        led_on(led);
    } else {
        led_off(led);
    }
    blink_led = !blink_led;
}

void state_machine(void) {
    geometry_msgs__msg__Twist v; // Msg to fill with cmd_vel

    // Check if the sensor are ok
    if (state == TAKING_OFF || state == HOVERING || state == EXTERNAL_CONTROL
        || state == LANDING || state == EMERGENCY_LANDING) {
        check_sensors();

        // New emergency, or a worse failure in the current one
        if (f_type != SENSORS_OK
            && (state != EMERGENCY_LANDING || f_type != f_emergency)) {
            start_emergency();
        }
    }

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
            if (get_take_off_ready()) {
                arm_motors(); // Allow the motors to receive velocities

                reset_h_reached(); // Reset the high checker

                change_state(TAKING_OFF);
            }
            break;

        case TAKING_OFF:
            // Add the desiered attitude of the drone, stable and h = target altitude
            att_target.cmd_vel = hover_vel;
            att_target.h = get_target_altitude();

            if (check_h_reached()) {
                led_on(LED_BLUE);

                reset_h_reached();

                change_state(HOVERING);
            } else {
                blink(LED_BLUE);
            }
            break;

        case HOVERING:
            att_target.cmd_vel = hover_vel;
            att_target.h = get_target_altitude();

            // Check if the drone has received a cmd_vel
            if (get_cmd_vel(&v, CMD_VEL_EXPIRE_TIME)) {
                change_state(EXTERNAL_CONTROL);
            }

            if (get_landing_ready()) {
                // Check if the drone has received the landing order
                led_on(LED_GREEN);
                led_off(LED_BLUE);

                reset_h_reached();

                change_state(LANDING);
            }
            break;

        case EXTERNAL_CONTROL:
            // Check if there is a new cmd_vel or if the time without cmd_vel has elapsed
            if (get_cmd_vel(&v, 500)) {
                att_target.cmd_vel = v;
                att_target.h = get_target_altitude();
            } else {
                att_target.cmd_vel = hover_vel;
                led_on(LED_BLUE);
                led_on(LED_GREEN);
                change_state(HOVERING);
            }

            // Check if the drone has received the landing order
            if (get_landing_ready()) {
                led_on(LED_GREEN);
                led_off(LED_BLUE);

                reset_h_reached();

                change_state(LANDING);
            } else {
                blink(LED_GREEN);
            }
            break;

        case LANDING:
            att_target.cmd_vel = hover_vel;
            att_target.h = 0;

            if (check_h_reached()) {
                led_on(LED_RED);
                led_on(LED_GREEN);
                led_on(LED_BLUE);

                reset_h_reached();

                change_state(DISARMING);
            } else {
                blink(LED_RED);
            }
            break;

        case DISARMING:
            disarm_motors(); // Block the motors
            motors_stop_all(); // Stop the motors as fallback

            led_off(LED_GREEN);
            led_off(LED_RED);
            led_off(LED_BLUE);

            change_state(CHECKING);
            break;

        case EMERGENCY_LANDING:
            att_target.cmd_vel = hover_vel;
            blink(LED_RED);
            blink(LED_GREEN);
            blink(LED_BLUE);

            if (f_type == IMU_FAILURE) {
                // Also in simulation: if only the IMU topic stops, the motor
                // speeds still arrive. If the link is lost they do not
                // arrive, but it ends in ERROR after the ramp anyway
                imu_cut_step(); // Low motors power step by step
            } else if (f_type == LINK_FAILURE) {
                link_land_step(); // Landing with the sensors, without ROS
            } else if (esp_timer_get_time() - t_emergency_us >= emergency_time_us) { // BARO_FAILURE
                finish_emergency(); // Emergency landing finished -> ERROR
            }
            break;

        case ERROR:
            led_on(LED_RED);
            led_off(LED_GREEN);
            led_off(LED_BLUE);
            break;
    }
}


static void system_task(void *arg) {
    (void)arg;

    static task_period_t period = TASK_PERIOD("system_task", SYSTEM_TASK_PERIOD_MS);
    TickType_t t_wake = xTaskGetTickCount();

    while (1) {
        task_period_tick(&period); // CPU / period log (CONFIG_CPU_STATS_ON)
        state_machine();

        // If the task was late, count again from now instead of running to catch up
        if (xTaskDelayUntil(&t_wake, pdMS_TO_TICKS(SYSTEM_TASK_PERIOD_MS)) == pdFALSE) {
            t_wake = xTaskGetTickCount();
        }
    }
}

void system_start(void) {
    xTaskCreate(system_task,
        "system_task",
        CONFIG_SYSTEM_TASK_STACK,
        NULL,
        CONFIG_SYSTEM_TASK_PRIO,
        NULL);
}
