// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_msgs:msg/Params.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/msg/params.h"


#ifndef MY_MSGS__MSG__DETAIL__PARAMS__STRUCT_H_
#define MY_MSGS__MSG__DETAIL__PARAMS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Params in the package my_msgs.
typedef struct my_msgs__msg__Params
{
  float h_max;
  float v_max;
  bool land_on_site;
} my_msgs__msg__Params;

// Struct for a sequence of my_msgs__msg__Params.
typedef struct my_msgs__msg__Params__Sequence
{
  my_msgs__msg__Params * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_msgs__msg__Params__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_MSGS__MSG__DETAIL__PARAMS__STRUCT_H_
