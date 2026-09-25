#ifndef ROS_COORDINATOR_H
#define ROS_COORDINATOR_H

#include <stdbool.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

#include "sdkconfig.h"


void ros_init(void);
bool ros_test(void);

// take off status
extern bool take_off_ready ;
extern float altitude;
bool get_take_off_ready(); // return the if srv arrive
float get_take_off_alt(); // return the altitude
geometry_msgs__msg__Twist get_cmd_vel(); // return the cmd_vel

#ifdef CONFIG_SIMULATION_ON
    void pub_motor_speed(uint8_t motor_id, uint8_t motor_spd);
    sensor_msgs__msg__Imu get_imu_sim();

#endif


#endif // ROS_COORDINATOR_H
