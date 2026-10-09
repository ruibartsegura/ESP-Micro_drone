/**
 * Made by Rui B.S.
 * Date: 29/07/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Public interface and data types of the attitude controller.
 *
 * Functions:
 *   - check_h_reached(): returns true if the drone is at the target height.
 *   - control_attitude(): runs one step of the controller.
 *   - init_attitude_controller(): creates the attitude task.
 *   - get_gains() / set_gains(): read / change the gains at runtime.
 */

#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include <stdbool.h>
#include <geometry_msgs/msg/twist.h>
#include "sdkconfig.h"


typedef struct roll_pitch_yaw {
    // Valores convertidos a unidades físicas
    float roll;
    float pitch;
    float yaw;
    int64_t t_stamp;
} RPY;

typedef struct attitude_target {
    geometry_msgs__msg__Twist cmd_vel;
    float h;
} ATTITUDE_TARGET;


// Gains of the controller that can be changed at runtime
typedef struct gains {
    float kp_h;        // m error -> motor power
    float kd_h;        // m error -> motor power
    float kp_angle;    // deg error -> deg/s target
    float kp_rate;     // deg/s error -> motor power
    float max_rate;    // max target roll/pitch rate (deg/s)
    float max_pow_rpy; // max power of each roll/pitch/yaw term
    float base;        // hover throttle
} GAINS;


bool check_h_reached();

void get_gains(GAINS *out);

#ifdef CONFIG_GAINS_TUNE_ON
    void set_gains(const GAINS *in);
#endif
void control_attitude();

// Start the task
void init_attitude_controller();

#endif // ATTITUDE_CONTROLLER_H
