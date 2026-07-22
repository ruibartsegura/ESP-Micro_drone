// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_msgs:srv/Takeoff.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/srv/takeoff.h"


#ifndef MY_MSGS__SRV__DETAIL__TAKEOFF__STRUCT_H_
#define MY_MSGS__SRV__DETAIL__TAKEOFF__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Takeoff in the package my_msgs.
typedef struct my_msgs__srv__Takeoff_Request
{
  /// Target altitude
  float altitude;
} my_msgs__srv__Takeoff_Request;

// Struct for a sequence of my_msgs__srv__Takeoff_Request.
typedef struct my_msgs__srv__Takeoff_Request__Sequence
{
  my_msgs__srv__Takeoff_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_msgs__srv__Takeoff_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Takeoff in the package my_msgs.
typedef struct my_msgs__srv__Takeoff_Response
{
  bool accepted;
  /// OK or why was denied
  rosidl_runtime_c__String reason;
} my_msgs__srv__Takeoff_Response;

// Struct for a sequence of my_msgs__srv__Takeoff_Response.
typedef struct my_msgs__srv__Takeoff_Response__Sequence
{
  my_msgs__srv__Takeoff_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_msgs__srv__Takeoff_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  my_msgs__srv__Takeoff_Event__request__MAX_SIZE = 1
};
// response
enum
{
  my_msgs__srv__Takeoff_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Takeoff in the package my_msgs.
typedef struct my_msgs__srv__Takeoff_Event
{
  service_msgs__msg__ServiceEventInfo info;
  my_msgs__srv__Takeoff_Request__Sequence request;
  my_msgs__srv__Takeoff_Response__Sequence response;
} my_msgs__srv__Takeoff_Event;

// Struct for a sequence of my_msgs__srv__Takeoff_Event.
typedef struct my_msgs__srv__Takeoff_Event__Sequence
{
  my_msgs__srv__Takeoff_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_msgs__srv__Takeoff_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_MSGS__SRV__DETAIL__TAKEOFF__STRUCT_H_
