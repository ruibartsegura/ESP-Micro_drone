// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from my_msgs:srv/Takeoff.idl
// generated code does not contain a copyright notice

#include "my_msgs/srv/detail/takeoff__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_my_msgs
const rosidl_type_hash_t *
my_msgs__srv__Takeoff__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x86, 0x09, 0xe8, 0x35, 0x61, 0xa6, 0xf5, 0x31,
      0xa6, 0xe0, 0xf1, 0x17, 0x69, 0x68, 0x6c, 0x55,
      0xe4, 0x14, 0x8b, 0xf2, 0xc2, 0x6d, 0x9d, 0x5c,
      0xfa, 0x3d, 0x2d, 0xba, 0x0e, 0xd4, 0x75, 0x25,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_my_msgs
const rosidl_type_hash_t *
my_msgs__srv__Takeoff_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x88, 0x49, 0xe9, 0xb4, 0x8d, 0x91, 0xff, 0x28,
      0xfd, 0xe1, 0xb7, 0xdb, 0x01, 0xa6, 0x6f, 0x40,
      0xa2, 0x2c, 0x6c, 0x06, 0x5f, 0xa5, 0x4a, 0xec,
      0xd1, 0x2a, 0x80, 0xfe, 0xdf, 0xff, 0xe1, 0x42,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_my_msgs
const rosidl_type_hash_t *
my_msgs__srv__Takeoff_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb8, 0xca, 0x16, 0x54, 0x39, 0xf3, 0xa3, 0x41,
      0xfe, 0xbf, 0x37, 0xda, 0x11, 0x82, 0x7a, 0x38,
      0xf5, 0xee, 0x19, 0x8a, 0x57, 0x77, 0x3e, 0x80,
      0xed, 0x62, 0xe4, 0x09, 0x94, 0xc6, 0x29, 0x21,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_my_msgs
const rosidl_type_hash_t *
my_msgs__srv__Takeoff_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x63, 0x7a, 0x7e, 0xcc, 0x98, 0x4d, 0x0a, 0xb3,
      0x3b, 0x2b, 0xe8, 0x45, 0x02, 0xaa, 0x46, 0x83,
      0x03, 0xee, 0x27, 0x1f, 0x3a, 0x62, 0x95, 0x02,
      0x48, 0x5d, 0xb8, 0x42, 0x81, 0x58, 0x07, 0x7f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char my_msgs__srv__Takeoff__TYPE_NAME[] = "my_msgs/srv/Takeoff";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char my_msgs__srv__Takeoff_Event__TYPE_NAME[] = "my_msgs/srv/Takeoff_Event";
static char my_msgs__srv__Takeoff_Request__TYPE_NAME[] = "my_msgs/srv/Takeoff_Request";
static char my_msgs__srv__Takeoff_Response__TYPE_NAME[] = "my_msgs/srv/Takeoff_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char my_msgs__srv__Takeoff__FIELD_NAME__request_message[] = "request_message";
static char my_msgs__srv__Takeoff__FIELD_NAME__response_message[] = "response_message";
static char my_msgs__srv__Takeoff__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field my_msgs__srv__Takeoff__FIELDS[] = {
  {
    {my_msgs__srv__Takeoff__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {my_msgs__srv__Takeoff_Request__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {my_msgs__srv__Takeoff__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {my_msgs__srv__Takeoff_Response__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
  {
    {my_msgs__srv__Takeoff__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {my_msgs__srv__Takeoff_Event__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription my_msgs__srv__Takeoff__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {my_msgs__srv__Takeoff_Event__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {my_msgs__srv__Takeoff_Request__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {my_msgs__srv__Takeoff_Response__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_msgs__srv__Takeoff__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_msgs__srv__Takeoff__TYPE_NAME, 19, 19},
      {my_msgs__srv__Takeoff__FIELDS, 3, 3},
    },
    {my_msgs__srv__Takeoff__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = my_msgs__srv__Takeoff_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = my_msgs__srv__Takeoff_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = my_msgs__srv__Takeoff_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char my_msgs__srv__Takeoff_Request__FIELD_NAME__altitude[] = "altitude";

static rosidl_runtime_c__type_description__Field my_msgs__srv__Takeoff_Request__FIELDS[] = {
  {
    {my_msgs__srv__Takeoff_Request__FIELD_NAME__altitude, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_msgs__srv__Takeoff_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_msgs__srv__Takeoff_Request__TYPE_NAME, 27, 27},
      {my_msgs__srv__Takeoff_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char my_msgs__srv__Takeoff_Response__FIELD_NAME__accepted[] = "accepted";
static char my_msgs__srv__Takeoff_Response__FIELD_NAME__reason[] = "reason";

static rosidl_runtime_c__type_description__Field my_msgs__srv__Takeoff_Response__FIELDS[] = {
  {
    {my_msgs__srv__Takeoff_Response__FIELD_NAME__accepted, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_msgs__srv__Takeoff_Response__FIELD_NAME__reason, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_msgs__srv__Takeoff_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_msgs__srv__Takeoff_Response__TYPE_NAME, 28, 28},
      {my_msgs__srv__Takeoff_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char my_msgs__srv__Takeoff_Event__FIELD_NAME__info[] = "info";
static char my_msgs__srv__Takeoff_Event__FIELD_NAME__request[] = "request";
static char my_msgs__srv__Takeoff_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field my_msgs__srv__Takeoff_Event__FIELDS[] = {
  {
    {my_msgs__srv__Takeoff_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {my_msgs__srv__Takeoff_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {my_msgs__srv__Takeoff_Request__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {my_msgs__srv__Takeoff_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {my_msgs__srv__Takeoff_Response__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription my_msgs__srv__Takeoff_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {my_msgs__srv__Takeoff_Request__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {my_msgs__srv__Takeoff_Response__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_msgs__srv__Takeoff_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_msgs__srv__Takeoff_Event__TYPE_NAME, 25, 25},
      {my_msgs__srv__Takeoff_Event__FIELDS, 3, 3},
    },
    {my_msgs__srv__Takeoff_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = my_msgs__srv__Takeoff_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = my_msgs__srv__Takeoff_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Request\n"
  "float32 altitude    # Target altitude\n"
  "---\n"
  "# Response\n"
  "bool accepted\n"
  "string reason       # OK or why was denied";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
my_msgs__srv__Takeoff__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_msgs__srv__Takeoff__TYPE_NAME, 19, 19},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 119, 119},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
my_msgs__srv__Takeoff_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_msgs__srv__Takeoff_Request__TYPE_NAME, 27, 27},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
my_msgs__srv__Takeoff_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_msgs__srv__Takeoff_Response__TYPE_NAME, 28, 28},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
my_msgs__srv__Takeoff_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_msgs__srv__Takeoff_Event__TYPE_NAME, 25, 25},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_msgs__srv__Takeoff__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_msgs__srv__Takeoff__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *my_msgs__srv__Takeoff_Event__get_individual_type_description_source(NULL);
    sources[3] = *my_msgs__srv__Takeoff_Request__get_individual_type_description_source(NULL);
    sources[4] = *my_msgs__srv__Takeoff_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_msgs__srv__Takeoff_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_msgs__srv__Takeoff_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_msgs__srv__Takeoff_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_msgs__srv__Takeoff_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_msgs__srv__Takeoff_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_msgs__srv__Takeoff_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *my_msgs__srv__Takeoff_Request__get_individual_type_description_source(NULL);
    sources[3] = *my_msgs__srv__Takeoff_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
