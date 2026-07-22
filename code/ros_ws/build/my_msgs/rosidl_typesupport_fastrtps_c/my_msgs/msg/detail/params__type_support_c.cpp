// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from my_msgs:msg/Params.idl
// generated code does not contain a copyright notice
#include "my_msgs/msg/detail/params__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "my_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "my_msgs/msg/detail/params__struct.h"
#include "my_msgs/msg/detail/params__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _Params__ros_msg_type = my_msgs__msg__Params;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_msgs
bool cdr_serialize_my_msgs__msg__Params(
  const my_msgs__msg__Params * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: h_max
  {
    cdr << ros_message->h_max;
  }

  // Field name: v_max
  {
    cdr << ros_message->v_max;
  }

  // Field name: land_on_site
  {
    cdr << (ros_message->land_on_site ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_msgs
bool cdr_deserialize_my_msgs__msg__Params(
  eprosima::fastcdr::Cdr & cdr,
  my_msgs__msg__Params * ros_message)
{
  // Field name: h_max
  {
    cdr >> ros_message->h_max;
  }

  // Field name: v_max
  {
    cdr >> ros_message->v_max;
  }

  // Field name: land_on_site
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->land_on_site = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_msgs
size_t get_serialized_size_my_msgs__msg__Params(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Params__ros_msg_type * ros_message = static_cast<const _Params__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: h_max
  {
    size_t item_size = sizeof(ros_message->h_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: v_max
  {
    size_t item_size = sizeof(ros_message->v_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: land_on_site
  {
    size_t item_size = sizeof(ros_message->land_on_site);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_msgs
size_t max_serialized_size_my_msgs__msg__Params(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: h_max
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: v_max
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: land_on_site
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = my_msgs__msg__Params;
    is_plain =
      (
      offsetof(DataType, land_on_site) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_msgs
bool cdr_serialize_key_my_msgs__msg__Params(
  const my_msgs__msg__Params * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: h_max
  {
    cdr << ros_message->h_max;
  }

  // Field name: v_max
  {
    cdr << ros_message->v_max;
  }

  // Field name: land_on_site
  {
    cdr << (ros_message->land_on_site ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_msgs
size_t get_serialized_size_key_my_msgs__msg__Params(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Params__ros_msg_type * ros_message = static_cast<const _Params__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: h_max
  {
    size_t item_size = sizeof(ros_message->h_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: v_max
  {
    size_t item_size = sizeof(ros_message->v_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: land_on_site
  {
    size_t item_size = sizeof(ros_message->land_on_site);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_msgs
size_t max_serialized_size_key_my_msgs__msg__Params(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: h_max
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: v_max
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: land_on_site
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = my_msgs__msg__Params;
    is_plain =
      (
      offsetof(DataType, land_on_site) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _Params__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const my_msgs__msg__Params * ros_message = static_cast<const my_msgs__msg__Params *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_my_msgs__msg__Params(ros_message, cdr);
}

static bool _Params__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  my_msgs__msg__Params * ros_message = static_cast<my_msgs__msg__Params *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_my_msgs__msg__Params(cdr, ros_message);
}

static uint32_t _Params__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_my_msgs__msg__Params(
      untyped_ros_message, 0));
}

static size_t _Params__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_my_msgs__msg__Params(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Params = {
  "my_msgs::msg",
  "Params",
  _Params__cdr_serialize,
  _Params__cdr_deserialize,
  _Params__get_serialized_size,
  _Params__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _Params__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Params,
  get_message_typesupport_handle_function,
  &my_msgs__msg__Params__get_type_hash,
  &my_msgs__msg__Params__get_type_description,
  &my_msgs__msg__Params__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, my_msgs, msg, Params)() {
  return &_Params__type_support;
}

#if defined(__cplusplus)
}
#endif
