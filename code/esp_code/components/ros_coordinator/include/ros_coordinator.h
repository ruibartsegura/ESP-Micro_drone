/**
 * Made by Rui B.S.
 * Date: 23/05/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Public interface of the micro-ROS coordinator. Other modules use it
 *   to read the commands that arrive from ROS 2.
 *
 * Functions:
 *   - ros_init(): starts the network interface and the micro-ROS task.
 *   - ros_test(): checks that the module started correctly.
 *   - get_take_off_ready(): returns true if a take-off request was accepted.
 *   - get_take_off_alt(): returns the requested take-off altitude.
 *   - get_cmd_vel(): returns the last velocity command.
 *   - pub_motor_speed(): (simulation) publishes the motor speeds.
 */

#ifndef ROS_COORDINATOR_H
#define ROS_COORDINATOR_H

#include <stdbool.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/pose_stamped.h>

#include "sdkconfig.h"


void ros_init(void);
bool ros_test(void);

// take off status
extern bool take_off_ready ;
extern float altitude;
bool get_take_off_ready(); // return if service has arrived
float get_take_off_alt(); // return the altitude

bool get_landing_ready(); // return service has arrived

bool get_new_vel();
geometry_msgs__msg__Twist get_cmd_vel(); // return the cmd_vel

#define NUM_MOTORS 4

#ifdef CONFIG_SIMULATION_ON
    void pub_motor_speed(const double power[NUM_MOTORS]);
#endif


#endif // ROS_COORDINATOR_H
