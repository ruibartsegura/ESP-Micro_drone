/**
 * Made by Rui B.S.
 * Date: 24/09/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Odometry estimator of the drone. It runs in its own FreeRTOS task
 *   every 10 ms. It estimates the orientation (Madgwick) and the horizontal
 *   velocity and position integrating the accelerometer in world axes. The
 *   velocity "forgets" with V_LEAK_TAU (leaky integration): with only an
 *   IMU the drift has no limit, so only the recent movements are kept
 *   (enough to brake a drift in the velocity loop).
 *
 * Functions:
 *   - estimate_odom(): estimates the orientation, velocity and position.
 *   - odom_task(): FreeRTOS task that calls estimate_odom() periodically.
 *   - odom_estimator_init(): creates the odometry task.
 *   - odom_estimator_test(): checks that the module started correctly.
 */

#include <stdbool.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "odometry_estimator.h"
#include "quaternions.h"
#include "attitude_controller.h"
#include "state.h"
#include "cpu_stats.h"

#include "sdkconfig.h"

#define ODOM_TASK_PERIOD_MS 10

#define g 9.81f // Gravity (m/s^2)

// Time constant of the leaky integration of the velocity (s). A bias of
// the accelerometer of b m/s^2 gives only b * V_LEAK_TAU m/s of error
#define V_LEAK_TAU 2.5f

static quat_t q;
static double last_t_imu;

static float vel_x, vel_y, vel_z;
static float x, y;

static bool is_init = false;

static const char *TAG = "estimate_odom_task";

void estimate_odom() {
    int64_t t_imu;
    get_time_imu(&t_imu);

    // Time difference
    float dt = (t_imu - last_t_imu) / 1000000000.0f; // (ns->s)
    last_t_imu = t_imu;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.0f;
    }

    vec3_t acc_lin = {0};
    get_acc_lin(&acc_lin);

    vec3_t vel_ang = {0};
    get_vel_ang(&vel_ang);

    // Quaternions use rad
    vel_ang.x *= M_PI / 180;
    vel_ang.y *= M_PI / 180;
    vel_ang.z *= M_PI / 180;

    // Get quaternion
    quaternion_update(&q,
                      vel_ang.x, vel_ang.y, vel_ang.z,
                      acc_lin.x, acc_lin.y, acc_lin.z,
                      dt);
    // set quaternion in the state
    set_orientation(q.x, q.y, q.z, q.w);

    // Get velocities of x & y
    float acc_x_world = (1 - 2*(q.y*q.y + q.z*q.z)) * acc_lin.x
                      + 2*(q.x*q.y - q.w*q.z)       * acc_lin.y
                      + 2*(q.x*q.z + q.w*q.y)       * acc_lin.z;

    float acc_y_world = 2*(q.x*q.y + q.w*q.z)       * acc_lin.x
                      + (1 - 2*(q.x*q.x + q.z*q.z)) * acc_lin.y
                      + 2*(q.y*q.z - q.w*q.x)       * acc_lin.z;

    // x & y without gravity
    float ax_w = acc_x_world * g;
    float ay_w = acc_y_world * g;

    // Leaky integration: the velocity goes back to 0 with V_LEAK_TAU
    vel_x += (ax_w - vel_x / V_LEAK_TAU) * dt;
    vel_y += (ay_w - vel_y / V_LEAK_TAU) * dt;
    vel_z = get_vel_z();
    set_vel_lin(vel_x, vel_y, vel_z);

    x += vel_x * dt;
    y += vel_y * dt;
    set_position(x, y);
}

void odom_task(void *arg) {
    (void)arg;

    // int log_counter = 0;

    static task_period_t period = TASK_PERIOD("odom_task", ODOM_TASK_PERIOD_MS);

    while (1) {
        task_period_tick(&period); // CPU / period log (CONFIG_CPU_STATS_ON)
        estimate_odom();

        // if (++log_counter >= 500) {
            // log_counter = 0;
            // UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
            // ESP_LOGI(TAG, "stack libre (min historico): %u bytes",
                     // (unsigned)(free_words * sizeof(StackType_t)));
        // }

        vTaskDelay(pdMS_TO_TICKS(ODOM_TASK_PERIOD_MS));
    }


}

void init_odom_estimator(void) {
    if (is_init) {
        return;
    }

    last_t_imu = 0;

    vel_x = 0;
    vel_y = 0;
    vel_z = 0;

    quaternion_init(&q);

    xTaskCreate(odom_task,
        "odom_task",
        CONFIG_ODOM_TASK_STACK,
        NULL,
        CONFIG_ODOM_TASK_PRIO,
        NULL);

    is_init = true;
}

bool odom_estimator_test(void) {
    if (!is_init) {
        return false;
    }

    return true;
}
