/**
 * Made by Rui B.S.
 * Date: 29/07/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Attitude controller of the drone. It runs a dual-loop cascade PID in
 *   its own FreeRTOS task every ATTITUDE_TASK_PERIOD_MS (5 ms in the drone,
 *   15 ms in simulation):
 *     - Outer loop: velocity loop. The error between cmd_vel and the
 *       velocity of the odometry gives the target roll / pitch, so a drift
 *       is braked (the drone tilts against it) and then it levels.
 *     - Inner loop: converts the angular rate error into motor commands
 *       with an X-frame motor mix (PI in roll / pitch, P in yaw).
 *   The height is controlled apart, adding the altitude error to the base
 *   throttle. Roll and pitch are estimated with a complementary filter, and
 *   the height with a second order complementary filter.
 *
 * Functions:
 *   - check_h_reached(): returns true if the drone is at the target height.
 *   - complementary_filter(): mixes two values with the alpha weight.
 *   - alpha_of(): weight of the gyro for a time step (time constant TAU_RP).
 *   - get_h(): estimates the height over the ground and the vertical
 *     velocity (accelerometer + barometer).
 *   - get_roll_pitch(): estimates roll and pitch from the IMU.
 *   - cmd_vel_2_RP(): velocity loop, converts the velocity error into
 *     target roll and pitch.
 *   - control_attitude(): runs one step of the controller and sets the motors.
 *   - attitude_task(): FreeRTOS task that calls control_attitude() periodically.
 *   - init_attitude_controller(): creates the attitude task.
 *   - get_gains() / set_gains(): read / change the gains while flying
 *     (used by the tuning task).
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
#include "cpu_stats.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "sdkconfig.h"


// Period of the controller. In simulation the IMU of Gazebo and the motor
// commands go at ~66 Hz (more is useless); in the drone they are on the
// board, so the controller goes at 200 Hz
#ifdef CONFIG_SIMULATION_ON
    #define ATTITUDE_TASK_PERIOD_MS 15 // ~66 Hz
#else
    #define ATTITUDE_TASK_PERIOD_MS 5  // 200 Hz
#endif
// #define STACK_LOG_EVERY (3750 / ATTITUDE_TASK_PERIOD_MS) // Cycles (~3.75 s)

// Complementary filter of roll / pitch: time constant (s) of the correction
// with the accelerometer. The weight of the gyro is calculated with the dt
// of every sample (alpha_of()), so it does not depend on the frequency.
// 0.735 s = the old ALPHA 0.98 at 15 ms
#define TAU_RP 0.735f

#define g 9.81f // Gravity (m/s^2)

#define MAX_ANGLE 0.35 // 20º in rad

// Limits so the attitude never takes all the throttle of the motors
#define MAX_RATE    200.0f // Max target roll/pitch rate (deg/s)
#define MAX_POW_RPY 400.0f // Max power of each roll/pitch/yaw term
#define MAX_POW_YAW 150.0f // Yaw has little torque: with more it only takes
                           // the range of the motors that roll / pitch need

#define THROTTLE_BASE 2378 // Min throttle to hover

// PID H
#define KP_H 100 // Proportional 350 pow / 1m error
#define KD_H 110 // Proportional 350 pow / 1m error

#define K_H_S 4.0f // Height correction (1/s)
#define K_V_S 4.0f // Vertical velocity correction (1/s^2)

#define KP_VEL 0.05f // m/s error -> target roll / pitch (rad), 2.9 deg per m/s

#define KP_ANGLE 2.0f   // deg error -> deg/s target
#define KP_RATE  0.3f   // deg/s error -> motor power

#define KI_RATE    0.6f
#define MAX_I_RATE 50.0f // Limit of the integral term (pow), ~5 x the CoM torque

#define KP_YAW_RATE 3.0f

#define PITCH_DEVIATION 10 // Value mesured for intial desviation
static bool was_flying = false;

// Gains used by the controller. They start with the defines and can be
// changed while flying with set_gains() (tuning task).
static GAINS gains = {
    .kp_h        = KP_H,
    .kd_h        = KD_H,
    .kp_vel      = KP_VEL,
    .kp_angle    = KP_ANGLE,
    .kp_rate     = KP_RATE,
    .ki_rate     = KI_RATE,
    .kp_yaw_rate = KP_YAW_RATE,
    .max_rate    = MAX_RATE,
    .max_pow_rpy = MAX_POW_RPY,
    .base        = THROTTLE_BASE,
};
static pthread_mutex_t gains_lock = PTHREAD_MUTEX_INITIALIZER;


// Allowed diference between target and actual height
#define DIFF_H_ALLOWED 0.05f // m
#define V_STABLE 0.05f // m/s, To be stable
#define TIME_H_STABLE_MS 500 // Time in the height with a stable velocity

static volatile bool h_reached = false;
static int64_t t_h_reached_us = -1;   // -1 = no started to count
static pthread_mutex_t h_lock = PTHREAD_MUTEX_INITIALIZER; // h_reached & t_h_reached_us

// Measures to keep
static RPY last_rpy;

static float h_est, last_vel_z;  // Estimated height (m) and vertical velocity (m/s)
static float h0;                 // Ground height (first barometer sample)
static int64_t last_t_h;         // IMU time of the last height estimation (ns)
static bool h_init = false;
static float err_h;

// Integral terms of the roll / pitch rate (pow). They only work in the flight
// states: in the others (ARMING, DISARMING...) they go back to 0
static float i_roll, i_pitch;
static int64_t last_t_ctrl;  // IMU time of the last control cycle (ns)

static bool is_init = false;
static const char *TAG = "ATTITUDE";

float get_vel_z() {
    return last_vel_z;
}


// ============================================================
//              Tune the gains of the controller
// ============================================================
void get_gains(GAINS *out) {
    pthread_mutex_lock(&gains_lock);
    *out = gains;
    pthread_mutex_unlock(&gains_lock);
}

#ifdef CONFIG_GAINS_TUNE_ON
void set_gains(const GAINS *in) {
    pthread_mutex_lock(&gains_lock);
    gains = *in;
    pthread_mutex_unlock(&gains_lock);
}
#endif


// ============================================================
//                  Checker of the height
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
//                   Auxiliar functions
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

// Weight of the gyro for a step of dt s: the accelerometer corrects with a
// time constant TAU_RP whatever the frequency is. dt = 0 (the same IMU
// sample again) gives 1: nothing new to correct with
static float alpha_of(float dt) {
    return TAU_RP / (TAU_RP + dt);
}


// ============================================================
//              Getting height & filtering it
// ============================================================
// Second order complementary filter: the accelerometer predicts the height
// and the vertical velocity, and the barometer corrects both.
// The height is relative to the ground (first barometer sample).
float get_h(float roll_deg, float pitch_deg) {
// The accelerometer is integrated -> time of the IMU (ns)
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
            return 0.0f;
        }
        h0 = pos.z;
        h_est = 0.0f;
        last_vel_z = 0.0f;
        last_t_h = t_imu;
        h_init = true;
        return h_est;
    }

    // Time difference
    float dt = (t_imu - last_t_h) / 1000000000.0f; // (ns->s)
    last_t_h = t_imu;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.0f;
    }

    // Vertical acceleration in world axes, without gravity, in m/s^2
    // Pitch & Roll (rad -> )
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

    // Correction with the barometer
    float err_baro = (pos.z - h0) - h_pred;
    h_est = h_pred + K_H_S * dt * err_baro;
    last_vel_z = vel_z_pred + K_V_S * dt * err_baro;

    return h_est;
}


// ============================================================
//                  Getting Roll Pitch Yaw
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

    float roll_acc  = atan2f(acc_lin.y, acc_lin.z)
                        * 180.0f / M_PI;

    float pitch_acc = atan2f(-acc_lin.x, sqrtf(acc_lin.y*acc_lin.y + acc_lin.z*acc_lin.z))
                        * 180.0f / M_PI;


    float roll_angle = (last_rpy.roll + vel_ang.x * dt);
    float pitch_angle = (last_rpy.pitch  + vel_ang.y * dt);

    // a = angle, b = angle acceleration
    float alpha = alpha_of(dt);
    *roll = complementary_filter(roll_angle, roll_acc, alpha);
    *pitch = complementary_filter(pitch_angle, pitch_acc, alpha);

    last_rpy.roll = *roll;
    last_rpy.pitch = *pitch;
    last_rpy.t_stamp = t_imu;
    return;
}


// ============================================================
//               CMD_VEL -> desired Roll/Pitch
// ============================================================
void cmd_vel_2_RP(float *targ_roll, float *targ_pitch, geometry_msgs__msg__Twist cmd_vel) {
    float roll, pitch;

    GAINS k;
    get_gains(&k);

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

    // ENU: +pitch pushes to +x and +roll pushes to -y
    roll = -clampf(k.kp_vel * err_vy, MAX_ANGLE);
    pitch = clampf(k.kp_vel * err_vx, MAX_ANGLE);

    *targ_roll = roll;
    *targ_pitch = pitch;
    return;
}


// ============================================================
//                        Attitude Main
// ============================================================
void control_attitude() {
    // External loop
    float roll, pitch;
    float targ_roll, targ_pitch;
    float err_roll, err_pitch;

    // Internal loop
    float roll_rate, pitch_rate, yaw_rate;
    float targ_roll_rate, targ_pitch_rate, targ_yaw_rate;
    float err_roll_rate, err_pitch_rate, err_yaw_rate;

    // Height
    float pow_h;

    // Power for the motors
    float pow_roll, pow_pitch, pow_yaw;

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
    sm_states_t sm;
    get_sm_state(&sm);
    bool flying = sm == TAKING_OFF || sm == HOVERING
                  || sm == EXTERNAL_CONTROL || sm == LANDING;

    if (flying && !was_flying) {
        // Take off: the integral starts with the trim, not with 0
        i_roll = 0;
        i_pitch = PITCH_DEVIATION;
    }
    was_flying = flying;

    if (flying) {
        i_roll = clampf(i_roll + k.ki_rate * err_roll_rate * dt, MAX_I_RATE);
        i_pitch = clampf(i_pitch + k.ki_rate * err_pitch_rate * dt, MAX_I_RATE);
    } else {
        i_roll = 0.0f;
        i_pitch = 0.0f;
    }

    // Calculate the power needed for the motors (PI in roll / pitch, P in yaw)
    pow_roll = clampf(k.kp_rate * err_roll_rate + i_roll, k.max_pow_rpy);
    pow_pitch = clampf(k.kp_rate * err_pitch_rate + i_pitch, k.max_pow_rpy);
    pow_yaw = clampf(k.kp_yaw_rate * err_yaw_rate, fminf(k.max_pow_rpy, MAX_POW_YAW));

    // ALTITUDE
    vec3_t pos = {0};
    get_position(&pos);

    float h = get_h(roll, pitch);

    err_h = attitude_target.h - h; // Distance between target and actual h.
    pow_h = err_h * k.kp_h - k.kd_h * last_vel_z;


    // Check if the drone has reached and hold the target H
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

    // Motors power.
    double power[N_MOTORS];
    power[0] = k.base + pow_h + pow_roll - pow_pitch - pow_yaw;
    power[1] = k.base + pow_h - pow_roll - pow_pitch + pow_yaw;
    power[2] = k.base + pow_h + pow_roll + pow_pitch + pow_yaw;
    power[3] = k.base + pow_h - pow_roll + pow_pitch - pow_yaw;

    // Set mottor speed (it clamps power[] to the real value sent)
    set_motor_speed(power);
}

static void attitude_task(void *arg) {
    (void)arg;
    // int log_counter = 0;

    static task_period_t period = TASK_PERIOD("attitude_task", ATTITUDE_TASK_PERIOD_MS);
    TickType_t t_wake = xTaskGetTickCount();

    while (1) {
        task_period_tick(&period); // CPU / period log (CONFIG_CPU_STATS_ON)
        control_attitude();

        // if (++log_counter >= STACK_LOG_EVERY) {
            // log_counter = 0;
            // UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
            // ESP_LOGI("system_task", "stack libre (min historico): %u bytes",
                        // (unsigned)(free_words * sizeof(StackType_t)));
        // }

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


// TODO:
//   *  Cambiar controll atitude para hacer "PID" en funciones
//
//   *  Hacer test para h, R, P, Y, motor final(rpm??)
