/**
 * Made by Rui B.S.
 * Date: 29/07/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Attitude controller of the drone. It runs a dual-loop cascade PID in
 *   its own FreeRTOS task every 15 ms:
 *     - Outer loop: converts cmd_vel into target roll/pitch angles with an
 *       aerodynamic drag model.
 *     - Inner loop: converts the angular rate error into motor commands
 *       with an X-frame motor mix.
 *   The height is controlled apart, adding the altitude error to the base
 *   throttle. Roll and pitch are estimated with a complementary filter, and
 *   the height with a second order complementary filter.
 *
 * Functions:
 *   - check_h_reached(): returns true if the drone is at the target height.
 *   - complementary_filter(): mixes two values with the ALPHA weight.
 *   - get_h(): estimates the height over the ground and the vertical
 *     velocity (accelerometer + barometer).
 *   - get_roll_pitch(): estimates roll and pitch from the IMU.
 *   - cmd_vel_2_RP(): converts a velocity command into target roll and pitch.
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

#include "esp_log.h"
#include "esp_timer.h"

#include "sdkconfig.h"


#define SYSTEM_TASK_PERIOD_MS 15 // 150HZ*

// Complementay
#define ALPHA 0.98 // Complementary filter constant

#define g 9.81f // Gravity (m/s^2)

#define MAX_ANGLE 0.35 // 20º in rad

// Limits so the attitude never takes all the throttle of the motors
#define MAX_RATE    200.0f // Max target roll/pitch rate (deg/s)
#define MAX_POW_RPY 400.0f // Max power of each roll/pitch/yaw term

#define THROTTLE_BASE 2378 // Min throttle to hover

// PID H
#define KP_H 100 // Proportional 350 pow / 1m error
#define KD_H 110 // Proportional 350 pow / 1m error

// Height filter (barometer correction, w ~ 2 rad/s with dt ~ 15 ms)
#define K_H 0.06f // Height correction
#define K_V 0.06f // Vertical velocity correction

// PID Angle & rate
#define KP_ANGLE 2   // deg error -> deg/s target
#define KP_RATE  3   // deg/s error -> motor power

// Gains used by the controller. They start with the defines and can be
// changed while flying with set_gains() (tuning task).
static GAINS gains = {
    .kp_h        = KP_H,
    .kd_h        = KD_H,
    .kp_angle    = KP_ANGLE,
    .kp_rate     = KP_RATE,
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
    h_est = h_pred + K_H * err_baro;
    last_vel_z = vel_z_pred + K_V * err_baro;

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
    *roll = complementary_filter(roll_angle, roll_acc, ALPHA);
    *pitch = complementary_filter(pitch_angle, pitch_acc, ALPHA);

    last_rpy.roll = *roll;
    last_rpy.pitch = *pitch;
    last_rpy.t_stamp = t_imu;
    return;
}


// ============================================================
//               CMD_VEL -> desired Roll/Pitch
// ============================================================
void cmd_vel_2_RP(float *targ_roll, float *targ_pitch, geometry_msgs__msg__Twist cmd_vel) {
    float roll, pitch, vx, vy;

    // Get input velocity
    vx = cmd_vel.linear.x;
    vy = cmd_vel.linear.y;

    // Get the roll and pitch for the input vel
    roll  = clampf(K_V * vy, MAX_ANGLE);
    pitch  = clampf(K_V * vx, MAX_ANGLE);

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

    // Calculate the power needed for the motors
    pow_roll = clampf(k.kp_rate * err_roll_rate, k.max_pow_rpy);
    pow_pitch = clampf(k.kp_rate * err_pitch_rate, k.max_pow_rpy);
    pow_yaw = clampf(k.kp_rate * err_yaw_rate, k.max_pow_rpy);

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
    int log_counter = 0;

    while (1) {
        control_attitude();

        if (++log_counter >= 250) {
            log_counter = 0;
            UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI("system_task", "stack libre (min historico): %u bytes",
                        (unsigned)(free_words * sizeof(StackType_t)));
        }

        vTaskDelay(pdMS_TO_TICKS(SYSTEM_TASK_PERIOD_MS));
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
