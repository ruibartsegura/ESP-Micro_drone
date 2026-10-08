/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Mock of the ROS 2 message types used by the firmware (host tests).
 *   The fields have the same names and types as the real generated C
 *   structs, so the production code compiles without changes. All the
 *   message headers (sensor_msgs/msg/imu.h, ...) include this file.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* ---- rosidl runtime ---- */
typedef struct { char *data; size_t size; size_t capacity; } rosidl_runtime_c__String;
typedef struct { double *data; size_t size; size_t capacity; } rosidl_runtime_c__double__Sequence;
bool rosidl_runtime_c__String__assign(rosidl_runtime_c__String *str, const char *value);
void rosidl_runtime_c__String__fini(rosidl_runtime_c__String *str);

/* ---- builtin_interfaces / std_msgs ---- */
typedef struct { int32_t sec; uint32_t nanosec; } builtin_interfaces__msg__Time;
typedef struct { builtin_interfaces__msg__Time stamp; rosidl_runtime_c__String frame_id; } std_msgs__msg__Header;
typedef struct { int32_t data; } std_msgs__msg__Int32;

/* ---- geometry_msgs ---- */
typedef struct { double x, y, z; }    geometry_msgs__msg__Vector3;
typedef struct { double x, y, z; }    geometry_msgs__msg__Point;
typedef struct { double x, y, z, w; } geometry_msgs__msg__Quaternion;
typedef struct { geometry_msgs__msg__Point position; geometry_msgs__msg__Quaternion orientation; } geometry_msgs__msg__Pose;
typedef struct { std_msgs__msg__Header header; geometry_msgs__msg__Pose pose; } geometry_msgs__msg__PoseStamped;
typedef struct { geometry_msgs__msg__Vector3 linear; geometry_msgs__msg__Vector3 angular; } geometry_msgs__msg__Twist;
typedef struct { std_msgs__msg__Header header; geometry_msgs__msg__Twist twist; } geometry_msgs__msg__TwistStamped;

/* ---- sensor_msgs ---- */
typedef struct {
    std_msgs__msg__Header          header;
    geometry_msgs__msg__Quaternion orientation;
    double                         orientation_covariance[9];
    geometry_msgs__msg__Vector3    angular_velocity;
    double                         angular_velocity_covariance[9];
    geometry_msgs__msg__Vector3    linear_acceleration;
    double                         linear_acceleration_covariance[9];
} sensor_msgs__msg__Imu;
bool sensor_msgs__msg__Imu__init(sensor_msgs__msg__Imu *msg);

typedef struct {
    std_msgs__msg__Header header;
    double fluid_pressure;
    double variance;
} sensor_msgs__msg__FluidPressure;

/* ---- my_msgs ---- */
typedef struct { float h_max; float v_max; bool land_on_site; } my_msgs__msg__Params;
typedef struct { float altitude; } my_msgs__srv__Takeoff_Request;
typedef struct { bool accepted; rosidl_runtime_c__String reason; } my_msgs__srv__Takeoff_Response;

/* ---- actuator_msgs ---- */
typedef struct {
    std_msgs__msg__Header              header;
    rosidl_runtime_c__double__Sequence position;
    rosidl_runtime_c__double__Sequence velocity;
    rosidl_runtime_c__double__Sequence normalized;
} actuator_msgs__msg__Actuators;
bool actuator_msgs__msg__Actuators__init(actuator_msgs__msg__Actuators *msg);
