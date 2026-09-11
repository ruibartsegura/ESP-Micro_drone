#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include <stdbool.h>

typedef struct roll_pitch_yaw {
    // Valores convertidos a unidades físicas
    float roll;
    float pitch;
    float yaw;
    int64_t t_stamp;
} RPY;

typedef struct roll_pitch_yaw_rate {
    // Valores convertidos a unidades físicas
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
} RPY_rate;

// Start the task
void init_attitude_controller();


#endif // ATTITUDE_CONTROLLER_H
