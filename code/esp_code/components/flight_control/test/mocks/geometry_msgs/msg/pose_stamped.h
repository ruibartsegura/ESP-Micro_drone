#pragma once
#include <stdint.h>
typedef struct { int32_t sec; uint32_t nanosec; } builtin_interfaces__msg__Time;
typedef struct { builtin_interfaces__msg__Time stamp; } std_msgs__msg__Header;
typedef struct { double x, y, z; } geometry_msgs__msg__Point;
typedef struct { geometry_msgs__msg__Point position; } geometry_msgs__msg__Pose;
typedef struct {
    std_msgs__msg__Header header;
    geometry_msgs__msg__Pose pose;
} geometry_msgs__msg__PoseStamped;
