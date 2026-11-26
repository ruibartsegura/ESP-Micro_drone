/**
 * Made by Rui B.S.
 * Date: 24/09/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Odometry estimator of the drone. It runs in its own FreeRTOS task
 *   every 10 ms. It estimates the orientation (Madgwick) and the horizontal
 *   velocity and position. The velocity comes from the rotor drag: in
 *   flight the accelerometer measures a_body = -K_DRAG * v_body, so it is
 *   not integrated and it does not drift. The yaw turns it to world axes
 *   and the position integrates it. K_DRAG is in parameters.c (it can
 *   change at runtime).
 *
 * Functions:
 *   - estimate_odom(): estimates the orientation, velocity and position.
 *   - odom_task(): FreeRTOS task that calls estimate_odom() periodically.
 *   - init_odom_estimator(): creates the odometry task.
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
#include "parameters.h"
#include "cpu_stats.h"

#include "sdkconfig.h"

#define ODOM_TASK_PERIOD_MS 10

#define g 9.81f // Gravity (m/s^2)

// Rotor drag: in flight the accelerometer does not see the gravity, only the
// thrust (Z) and the drag of the rotors, proportional to the velocity:
// a_body = -K_DRAG * v_body (rosbag 15/10: K_DRAG = 0.124 1/s, corr -1.00).
// K_DRAG: get_odom_params() (parameters.c, menuconfig x1000)
#define TAU_VEL 0.1f // Low pass of the velocity (s)

static quat_t q;
static double last_t_imu;

static float vel_x, vel_y, vel_z;
static float x, y;

static volatile uint32_t odom_iter = 0; // every loop of the task, for the test
#define TEST_ITER 5

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
    // Velocity in the axes of the drone. Only in flight: on the ground the
    // accelerometer sees the gravity of the tilt, not the drag
    sm_states_t sm;
    get_sm_state(&sm);

    bool flying = false;
    if (sm == TAKING_OFF || sm == HOVERING || sm == EXTERNAL_CONTROL || sm == LANDING
        || sm == EMERGENCY_LANDING) {
        flying = true;
    }

    // Body velocity in X & Y
    float vbx = 0.0f, vby = 0.0f;
    odom_params_t op;
    get_odom_params(&op);
    if (flying && op.k_drag > 0.0f) { // k_drag can change at runtime: avoid /0
        vbx = -acc_lin.x * g / op.k_drag; // acc_lin in g -> m/s^2
        vby = -acc_lin.y * g / op.k_drag;
    }

    // Get world velocities
    float yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y),
                       1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    float vx_w = cosf(yaw) * vbx - sinf(yaw) * vby;
    float vy_w = sinf(yaw) * vbx + cosf(yaw) * vby;

    // Low pass: the accelerometer is noisy
    // Out of flight it goes back to 0
    float beta = dt / (TAU_VEL + dt);
    vel_x += beta * (vx_w - vel_x);
    vel_y += beta * (vy_w - vel_y);
    vel_z = get_vel_z();
    set_vel_lin(vel_x, vel_y, vel_z);

    // Get X & Y
    x += vel_x * dt;
    y += vel_y * dt;
    set_position(x, y);
}

void odom_task(void *arg) {
    (void)arg;

    static task_period_t period = TASK_PERIOD("odom_task", ODOM_TASK_PERIOD_MS);

    while (1) {
        task_period_tick(&period); // CPU / period log (CONFIG_CPU_STATS_ON)
        estimate_odom();
        odom_iter++;

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

    uint32_t it0 = odom_iter;

    vTaskDelay(pdMS_TO_TICKS(100)); // ~10 iterations expected

    uint32_t it1 = odom_iter;

    // If less than the half there is a error
    if ((it1 - it0) < TEST_ITER) {
        return false;
    }
    return true;
}
