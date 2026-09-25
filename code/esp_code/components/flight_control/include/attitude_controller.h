#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include <stdbool.h>
#include <geometry_msgs/msg/twist.h>


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


bool check_h_reached();

void control_attitude();

// Start the task
void init_attitude_controller();

#endif // ATTITUDE_CONTROLLER_H
