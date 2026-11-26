/**
 * Made by Rui B.S.
 * Date: 29/07/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Public interface and data types of the attitude controller.
 *
 * Functions:
 *   - get_vel_z(): returns the last vertical velocity (used by the odometry).
 *   - get_h_est(): returns the last estimated height (used by the failsafe).
 *   - get_last_motor_power(): returns the mean power of the last motor command.
 *   - reset_h_reached(): clears the flag of target height reached.
 *   - check_h_reached(): returns true if the drone is at the target height.
 *   - control_attitude(): runs one step of the controller.
 *   - init_attitude_controller(): creates the attitude task.
 *
 *   The gains (GAINS, get_gains() / set_gains()) are in parameters.h.
 */

#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include <stdbool.h>
#include <geometry_msgs/msg/twist.h>
#include "sdkconfig.h"
#include "parameters.h" // GAINS, get_gains()


typedef struct roll_pitch_yaw {
    float roll;
    float pitch;
    float yaw;
    int64_t t_stamp;
} RPY;

typedef struct attitude_target {
    geometry_msgs__msg__Twist cmd_vel;
    float h;
} ATTITUDE_TARGET;


float get_vel_z(); // To avoid the calc in odometry
float get_h_est(void);
float get_last_motor_power(void);

void reset_h_reached();
bool check_h_reached();

void control_attitude();

// Start the task
void init_attitude_controller();
bool attitude_controller_test(void); // Check leds

#endif // ATTITUDE_CONTROLLER_H
