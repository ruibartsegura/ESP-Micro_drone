/**
 * Made by Rui B.S.
 * Date: 29/07/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Attitude controller of the drone. It runs a three-stage cascade in its
 *   own FreeRTOS task every ATTITUDE_TASK_PERIOD_MS (5 ms in the drone,
 *   15 ms in simulation): velocity -> angle -> angular rate.
 *     - Stage 1, velocity (P): the error between cmd_vel and the velocity
 *       of the odometry gives the target roll / pitch (limited to
 *       max_angle), so a drift is braked (the drone tilts against it) and
 *       then it levels.
 *     - Stage 2, angle (P): the roll / pitch error gives the target angular
 *       rate (limited to max_rate).
 *     - Stage 3, angular rate: the rate error gives the speed correction of
 *       each axis (PI in roll / pitch, P in yaw; yaw only has this stage),
 *       mixed into the target angular speed (rad/s) of each motor with an
 *       X-frame motor mix. The conversion to PWM is done in motors.c.
 *   The height is controlled apart (PD), adding the altitude correction to
 *   the base motor speed. Roll and pitch are estimated with a complementary filter, and
 *   the height with a second order complementary filter.
 *
 * Functions:
 *   - get_vel_z(): returns the last vertical velocity (used by the odometry).
 *   - get_h_est(): returns the last estimated height (used by the failsafe).
 *   - get_last_motor_power(): returns the mean angular speed (rad/s) of the
 *     last motor command.
 *   - reset_h_reached(): clears the flag of target height reached.
 *   - check_h_reached(): returns true if the drone is at the target height.
 *   - clampf(): limits a value to [-lim, lim].
 *   - complementary_filter(): mixes two values with the alpha weight.
 *   - get_h(): estimates the height over the ground and the vertical
 *     velocity (accelerometer + barometer).
 *   - get_roll_pitch(): estimates roll and pitch from the IMU.
 *   - limit_vel(): limits the module of the horizontal velocity to v_max.
 *   - cmd_vel_2_RP(): velocity loop, converts the velocity error into
 *     target roll and pitch.
 *   - control_attitude(): runs one step of the controller and sets the motors.
 *   - attitude_task(): FreeRTOS task that calls control_attitude() periodically.
 *   - init_attitude_controller(): creates the attitude task.
 *
 *   The gains and the limits are in parameters.c (get_gains()): ROS and the
 *   tuning task can change them at runtime.
 */

#include <math.h>
#include <pthread.h>

#include "attitude_controller.h"

#include "imu.h"
#include "height.h"
#include "system.h"
#include "ros_coordinator.h"
#include "motors.h"
#include "state.h"
#include "parameters.h"
#include "cpu_stats.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "sdkconfig.h"


// Period of the controller. In simulation the IMU of Gazebo and the motor
// commands go at ~66 Hz; in the drone they are on the
// board, so the controller goes at 200 Hz
#ifdef CONFIG_SIMULATION_ON
    #define ATTITUDE_TASK_PERIOD_MS 15 // ~66 Hz

#else
    #define ATTITUDE_TASK_PERIOD_MS 5  // 200 Hz

#endif

#define g 9.81f // Gravity (m/s^2)

// Gains, limits (max_rate, max_pow_rp, max_pow_yaw, max_angle), base speed,
// alpha of the complementary filter and pitch / roll deviation: parameters.c
// (get_gains()). The yaw limit is small: yaw has little torque and with more
// it only takes the range of the motors that roll / pitch need.

#define K_H_S 4.0f // Height correction (1/s)
#define K_V_S 4.0f // Vertical velocity correction (1/s^2)

#define MAX_I_RATE 50.0f // Limit of the integral term (rad/s), ~5 x the CoM torque

static bool was_flying = false;

// Allowed diference between target and actual height
#define DIFF_H_ALLOWED 0.05f // m
#define V_STABLE 0.05f // m/s, To be stable
#define TIME_H_STABLE_MS 500 // Time in the height with a stable velocity

static volatile bool h_reached = false;
static int64_t t_h_reached_us = -1; // -1 = no started to count
static pthread_mutex_t h_lock = PTHREAD_MUTEX_INITIALIZER;

// Measures to keep
static RPY last_rpy;

static float h_est, last_vel_z; // Estimated height (m) and vertical velocity (m/s)
static float h0; // Ground height (first barometer sample)
static int64_t last_t_h; // IMU time of the last height estimation (ns)
static bool h_init = false;
static float err_h;

// Integral terms of the roll / pitch rate (rad/s). They only work in the flight
// states: in the others (ARMING, DISARMING...) they go back to 0
static float i_roll, i_pitch;
static int64_t last_t_ctrl;  // IMU time of the last control cycle (ns)

// Emergency landing without barometer: the motor speed goes to a bit less than
// the hover one and stays there (open loop).
#define EMRGY_POW_DOWN 70.0f  // Below the hover speed (rad/s)
#define EMRGY_POW_RATE 140.0f // Speed of that decrease (rad/s^2)



#define HOVER_POW_TAU_S 2.0f // Time constant of the mean w_h in flight (s)

static float hover_w_h = 0.0f; // Mean w_h in flight: correction of k.base to hover (rad/s)
static float emrg_drop = 0.0f; // Speed already decreased in the emergency landing (rad/s)
static volatile float last_motor_power = 0.0f; // Mean angular speed of the last motor command (rad/s)

static volatile uint32_t att_iter = 0; // every loop of the task, for the test
#define TEST_ITER 5

static bool is_init = false;
static const char *TAG = "ATTITUDE";

float get_vel_z() {
    return last_vel_z;
}

float get_h_est(void) {
    return h_est;
}

float get_last_motor_power(void) {
    return last_motor_power;
}


// ============================================================
//                    Checker of the height
// ============================================================
void reset_h_reached() {
    pthread_mutex_lock(&h_lock);
    h_reached = false;
    t_h_reached_us = -1;
    pthread_mutex_unlock(&h_lock);
}

bool check_h_reached() {
    pthread_mutex_lock(&h_lock);
    bool reached = h_reached;
    pthread_mutex_unlock(&h_lock);
    return reached;
}


// ============================================================
//                      Auxiliar functions
// ============================================================
static float clampf(float x, float lim) {
    if (x > lim) {
        return lim;
    } else if (x < -lim) {
        return -lim;
    }
    return x;
}

static float complementary_filter(float a, float b, float alpha) {
    return (alpha * a + (1.0 - alpha) * b);
}


// ============================================================
//                Getting height & filtering it
// ============================================================
// Get the actual height. The height is relative to the ground (first barometer sample).
float get_h(float roll_deg, float pitch_deg) {
    // Get barometer ad IMU las sample time
    int64_t t_imu, t_baro;
    get_time_imu(&t_imu);
    get_time_height(&t_baro);

    vec3_t acc_lin = {0};
    get_acc_lin(&acc_lin);

    vec3_t pos = {0};
    get_position(&pos);

    // Wait for the first barometer sample to take the ground reference
    if (!h_init) {
        if (t_baro == 0) {
            return 0.0f; // No data then return 0 m
        }
        h0 = pos.z; // Use first sample as ground position
        h_est = 0.0f;
        last_vel_z = 0.0f;
        last_t_h = t_imu;
        h_init = true;
        return h_est;
    }

    // Time difference between actual sample and last time
    float dt = (t_imu - last_t_h) / 1000000000.0f; // (ns->s)
    last_t_h = t_imu;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.0f;
    }

    // Vertical acceleration in world axes, without gravity, in m/s^2
    // Pitch & Roll (rad -> deg)
    float r = roll_deg * M_PI / 180.0f;
    float p = pitch_deg * M_PI / 180.0f;

    // IMU gives acc_z with frame_id = drone_body
    // We get the acc_z with frame_id = world
    float acc_z_world = - sinf(p) * acc_lin.x
                        + sinf(r) * cosf(p) * acc_lin.y
                        + cosf(r) * cosf(p) * acc_lin.z;
    float acc_z = (acc_z_world - 1.0f) * g;

    // Prediction with the accelerometer
    float h_pred = h_est + last_vel_z * dt + 0.5f * acc_z * dt * dt;
    float vel_z_pred = last_vel_z + acc_z * dt;

    // The barometer data is frozen: do not correct with it
    if (get_barometer_failing()) {
        h_est = h_pred;
        last_vel_z = vel_z_pred;
        return h_est;
    }

    // Correction with the barometer
    float err_baro = (pos.z - h0) - h_pred;
    h_est = h_pred + K_H_S * dt * err_baro;
    last_vel_z = vel_z_pred + K_V_S * dt * err_baro;

    return h_est;
}


// ============================================================
//                      Getting Roll Pitch
// ============================================================
void get_roll_pitch(float *roll, float *pitch) {
    // Get the time of the sample
    int64_t t_imu;
    get_time_imu(&t_imu);

    vec3_t acc_lin = {0};
    vec3_t vel_ang = {0};

    get_acc_lin(&acc_lin);
    get_vel_ang(&vel_ang);

    float dt = (t_imu - last_rpy.t_stamp) / 1000000000.0f; // Time difference (ns->s)
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.0f;
    }

    float roll_acc  = atan2f(acc_lin.y, acc_lin.z) * 180.0f / M_PI;

    float pitch_acc = atan2f(-acc_lin.x, sqrtf(acc_lin.y*acc_lin.y + acc_lin.z*acc_lin.z))
                        * 180.0f / M_PI;


    // Calculate roll and pitch
    float roll_angle = (last_rpy.roll + vel_ang.x * dt);
    float pitch_angle = (last_rpy.pitch  + vel_ang.y * dt);

    // Filter roll and pitch (alpha can change at runtime)
    GAINS k;
    get_gains(&k);
    *roll = complementary_filter(roll_angle, roll_acc, k.alpha);
    *pitch = complementary_filter(pitch_angle, pitch_acc, k.alpha);

    last_rpy.roll = *roll;
    last_rpy.pitch = *pitch;
    last_rpy.t_stamp = t_imu;
    return;
}


// ============================================================
//                CMD_VEL -> desired Roll/Pitch
// ============================================================
// Limits the module of the horizontal velocity (x, y) to v_max. The vector
// is scaled, so the direction does not change
static void limit_vel(double *vx, double *vy, float v_max) {
    double v = sqrt(*vx * *vx + *vy * *vy);
    if (v > v_max && v > 0.0) {
        double scale = v_max / v;
        *vx *= scale;
        *vy *= scale;
    }
}

void cmd_vel_2_RP(float *targ_roll, float *targ_pitch, geometry_msgs__msg__Twist cmd_vel) {
    float roll, pitch;

    GAINS k;
    get_gains(&k);

    // The velocity asked by cmd_vel can not be higher than v_max (parameters)
    limit_vel(&cmd_vel.linear.x, &cmd_vel.linear.y, get_max_velocity());

    // Velocity of the odometry (world axes) turned to the axes of the drone
    // with the yaw, the same axes of cmd_vel
    vec3_t v = {0};
    get_vel_lin(&v);
    quat_t q = {0};
    get_orientation(&q);
    float yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y),
                       1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    float vx_body = cosf(yaw) * v.x + sinf(yaw) * v.y;
    float vy_body = -sinf(yaw) * v.x + cosf(yaw) * v.y;

    // Velocity error: in hover (cmd_vel = 0) it is the drift, so the drone
    // tilts against it until it stops and then levels
    float err_vx = cmd_vel.linear.x - vx_body;
    float err_vy = cmd_vel.linear.y - vy_body;

    roll = -clampf(k.kp_vel * err_vy, k.max_angle);
    pitch = clampf(k.kp_vel * err_vx, k.max_angle);

    *targ_roll = roll;
    *targ_pitch = pitch;
    return;
}


// ============================================================
//                        Attitude Main
// ============================================================
void control_attitude() {
    sm_states_t sm;
    get_sm_state(&sm);
    bool flying = false;
    bool baro_failing = false;

    // If there is a emergency landing -> 3 options
    // 1. IMU failing -> ignore attitude controller -> reduce the motor speed from system.c
    // 2. Barometer is failing -> we can land in a stabilized manner
    // 3. Link lost -> sensors ok: normal control to h = 0 (from system.c)
    if (sm == EMERGENCY_LANDING) {
        if (get_imu_failing()) {
            return;
        } else if (get_barometer_failing()) {
            baro_failing = true;
        }
    }

    if (sm == TAKING_OFF || sm == HOVERING || sm == EXTERNAL_CONTROL || sm == LANDING
        || sm == EMERGENCY_LANDING) {
        flying = true;
    }

    // External loop
    float roll, pitch;
    float targ_roll, targ_pitch;
    float err_roll, err_pitch;

    // Internal loop
    float roll_rate, pitch_rate, yaw_rate;
    float targ_roll_rate, targ_pitch_rate, targ_yaw_rate;
    float err_roll_rate, err_pitch_rate, err_yaw_rate;

    // Height correction of the motor speed (rad/s)
    float w_h;

    // Roll / pitch / yaw corrections of the motor speed (rad/s)
    float w_roll, w_pitch, w_yaw;

    // Get the desired attitude of the drone
    ATTITUDE_TARGET attitude_target = get_attitude();

    // Gains of this cycle (they can change at runtime)
    GAINS k;
    get_gains(&k);

    // EXTERNAL LOOP
    // Get Roll & Pitch
    get_roll_pitch(&roll, &pitch);

    // Get target roll & pitch
    cmd_vel_2_RP(&targ_roll, &targ_pitch, attitude_target.cmd_vel);

    // Get the error in the roll and pitch
    err_roll = targ_roll * 180.0f/M_PI - roll;
    err_pitch = targ_pitch * 180.0f/M_PI - pitch;


    // INTERNAL LOOP
    // Get the rate of roll & pitch
    targ_roll_rate = clampf(k.kp_angle * err_roll, k.max_rate);
    targ_pitch_rate = clampf(k.kp_angle * err_pitch, k.max_rate);
    targ_yaw_rate = attitude_target.cmd_vel.angular.z * 180.0f/M_PI;

    // Get measured roll, pitch, yaw rate
    vec3_t vel_ang = {0};
    get_vel_ang(&vel_ang);

    roll_rate = vel_ang.x;
    pitch_rate = vel_ang.y;
    yaw_rate = vel_ang.z;

    // Get diff between measured and desired
    err_roll_rate = targ_roll_rate - roll_rate;
    err_pitch_rate = targ_pitch_rate - pitch_rate;
    err_yaw_rate = targ_yaw_rate - yaw_rate;

    // Time of the cycle for the integral (IMU time, ns -> s). dt = 0 with
    // the same IMU sample: nothing new to add
    int64_t t_imu;
    get_time_imu(&t_imu);
    float dt = (t_imu - last_t_ctrl) / 1000000000.0f;
    last_t_ctrl = t_imu;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.0f;
    }

    // Integral only in the flight states (also taking off: the torque that
    // tilts the drone appears when it starts to leave the ground)
    if (flying && !was_flying) {
        // Take off: the integral starts with the trim, not with 0
        i_roll = k.roll_deviation;
        i_pitch = k.pitch_deviation;
    }
    was_flying = flying;

    if (flying) {
        i_roll = clampf(i_roll + k.ki_rate * err_roll_rate * dt, MAX_I_RATE);
        i_pitch = clampf(i_pitch + k.ki_rate * err_pitch_rate * dt, MAX_I_RATE);
    } else {
        i_roll = 0.0f;
        i_pitch = 0.0f;
    }

    // Calculate the speed correction needed in each axis (PI in roll / pitch, P in yaw)
    w_roll = clampf(k.kp_rate * err_roll_rate + i_roll, k.max_pow_rp);
    w_pitch = clampf(k.kp_rate * err_pitch_rate + i_pitch, k.max_pow_rp);
    w_yaw = clampf(k.kp_yaw_rate * err_yaw_rate, k.max_pow_yaw);

    // ALTITUDE
    if (!baro_failing) {
        float h = get_h(roll, pitch); // Also without barometer: the odometry uses the velocity
        err_h = attitude_target.h - h; // Distance between target and actual h.
        w_h = err_h * k.kp_h - k.kd_h * last_vel_z;

        // Mean w_h in stable flight (take off and landing are transients)
        if (sm == HOVERING || sm == EXTERNAL_CONTROL) {
            float k_hover = ATTITUDE_TASK_PERIOD_MS / 1000.0f / HOVER_POW_TAU_S;
            hover_w_h += k_hover * (w_h - hover_w_h);
        }

        // Check if the drone has reached and holded the target altitude
        pthread_mutex_lock(&h_lock);
        if (fabsf(err_h) <= DIFF_H_ALLOWED && fabsf(last_vel_z) <= V_STABLE) {
            if (t_h_reached_us < 0) {
                t_h_reached_us = esp_timer_get_time(); // First time in target
            }
            h_reached = (esp_timer_get_time() - t_h_reached_us) >= TIME_H_STABLE_MS * 1000LL;
        } else {
            t_h_reached_us = -1;
            h_reached = false;
        }
        pthread_mutex_unlock(&h_lock);
    } else {
        // Decrease the speed to a bit less than the hover one and hold it.
        // system.c stops the motors when the time of the landing has elapsed
        if (emrg_drop < EMRGY_POW_DOWN) {
            emrg_drop += EMRGY_POW_RATE * (ATTITUDE_TASK_PERIOD_MS / 1000.0f);
        }
        w_h = hover_w_h - emrg_drop;
    }

    // Target angular speed of each motor (rad/s). motors.c limits it and
    // converts it to PWM
    double omega[N_MOTORS];
    omega[0] = k.base + w_h + w_roll - w_pitch - w_yaw;
    omega[1] = k.base + w_h - w_roll - w_pitch + w_yaw;
    omega[2] = k.base + w_h + w_roll + w_pitch + w_yaw;
    omega[3] = k.base + w_h - w_roll + w_pitch - w_yaw;

    // Set motor speed
    set_motor_speed(omega);

    // Mean speed, start of the ramp if the IMU fails
    last_motor_power = (omega[0] + omega[1] + omega[2] + omega[3]) / N_MOTORS;
}

static void attitude_task(void *arg) {
    (void)arg;

    static task_period_t period = TASK_PERIOD("attitude_task", ATTITUDE_TASK_PERIOD_MS);
    TickType_t t_wake = xTaskGetTickCount();

    while (1) {
        task_period_tick(&period); // CPU / period log (CONFIG_CPU_STATS_ON)
        control_attitude();
        att_iter++;

        // Fixed period (vTaskDelay would add the time of the work). If the
        // task was late, count again from now instead of running to catch up
        if (xTaskDelayUntil(&t_wake, pdMS_TO_TICKS(ATTITUDE_TASK_PERIOD_MS)) == pdFALSE) {
            t_wake = xTaskGetTickCount();
        }
    }
}


void init_attitude_controller() {
    if (is_init) {
        return;
    }

    xTaskCreate(attitude_task,
        "attitude_task",
        CONFIG_ATTITUDE_TASK_STACK,
        NULL,
        CONFIG_ATTITUDE_TASK_PRIO,
        NULL);

    is_init = true;
}

bool attitude_controller_test(void) {
    if (!is_init) {
        return false;
    }

    uint32_t it0 = att_iter;

    vTaskDelay(pdMS_TO_TICKS(100)); // ~10 iterations expected

    uint32_t it1 = att_iter;

    // If less than the half there is a error
    if ((it1 - it0) < TEST_ITER) {
        return false;
    }
    return true;
}