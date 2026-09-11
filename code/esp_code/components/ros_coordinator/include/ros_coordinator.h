#ifndef ROS_COORDINATOR_H
#define ROS_COORDINATOR_H

#include <stdbool.h>
#include <geometry_msgs/msg/twist.h>


void ros_init(void);
bool ros_test(void);

// take off status
extern bool take_off_ready ;
extern float altitude;
bool get_take_off_ready(); // return the if srv arrive
float get_take_off_alt(); // return the altitude
geometry_msgs__msg__Twist get_cmd_vel(); // return the cmd_vel

#endif // ROS_COORDINATOR_H
