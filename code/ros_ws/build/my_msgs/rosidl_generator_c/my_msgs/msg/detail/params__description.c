// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from my_msgs:msg/Params.idl
// generated code does not contain a copyright notice

#include "my_msgs/msg/detail/params__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_my_msgs
const rosidl_type_hash_t *
my_msgs__msg__Params__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4a, 0xb7, 0x75, 0xb3, 0xab, 0x0d, 0xc5, 0xfa,
      0xda, 0x99, 0x74, 0xcd, 0xfd, 0x3a, 0x8c, 0x4f,
      0xb3, 0xcd, 0xc3, 0x18, 0xfc, 0xa6, 0x5d, 0x20,
      0xd5, 0x19, 0xf6, 0x8b, 0xa6, 0x47, 0x68, 0xab,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char my_msgs__msg__Params__TYPE_NAME[] = "my_msgs/msg/Params";

// Define type names, field names, and default values
static char my_msgs__msg__Params__FIELD_NAME__h_max[] = "h_max";
static char my_msgs__msg__Params__FIELD_NAME__v_max[] = "v_max";
static char my_msgs__msg__Params__FIELD_NAME__land_on_site[] = "land_on_site";

static rosidl_runtime_c__type_description__Field my_msgs__msg__Params__FIELDS[] = {
  {
    {my_msgs__msg__Params__FIELD_NAME__h_max, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_msgs__msg__Params__FIELD_NAME__v_max, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_msgs__msg__Params__FIELD_NAME__land_on_site, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_msgs__msg__Params__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_msgs__msg__Params__TYPE_NAME, 18, 18},
      {my_msgs__msg__Params__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 h_max\n"
  "float32 v_max\n"
  "bool land_on_site";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
my_msgs__msg__Params__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_msgs__msg__Params__TYPE_NAME, 18, 18},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 45, 45},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_msgs__msg__Params__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_msgs__msg__Params__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
