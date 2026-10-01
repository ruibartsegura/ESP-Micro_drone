#pragma once
#include <stdint.h>
typedef struct { double x, y, z; } geometry_msgs__msg__Vector3;
typedef struct {
    geometry_msgs__msg__Vector3 linear;
    geometry_msgs__msg__Vector3 angular;
} geometry_msgs__msg__Twist;
