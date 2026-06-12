// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_nav_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

#include "robot_nav_interfaces/msg/detail/detection__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_nav_interfaces
const rosidl_type_hash_t *
robot_nav_interfaces__msg__Detection__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa3, 0xcc, 0x84, 0xd9, 0x36, 0xcb, 0x64, 0xde,
      0xac, 0xb9, 0x8c, 0x7f, 0x1e, 0xcf, 0x6e, 0xc8,
      0xea, 0x89, 0xc3, 0x4a, 0x1f, 0x7a, 0x97, 0x31,
      0x80, 0x3f, 0xe5, 0xef, 0x96, 0x8f, 0x26, 0x4a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char robot_nav_interfaces__msg__Detection__TYPE_NAME[] = "robot_nav_interfaces/msg/Detection";

// Define type names, field names, and default values
static char robot_nav_interfaces__msg__Detection__FIELD_NAME__class_name[] = "class_name";
static char robot_nav_interfaces__msg__Detection__FIELD_NAME__x_min[] = "x_min";
static char robot_nav_interfaces__msg__Detection__FIELD_NAME__y_min[] = "y_min";
static char robot_nav_interfaces__msg__Detection__FIELD_NAME__x_max[] = "x_max";
static char robot_nav_interfaces__msg__Detection__FIELD_NAME__y_max[] = "y_max";
static char robot_nav_interfaces__msg__Detection__FIELD_NAME__confidence[] = "confidence";
static char robot_nav_interfaces__msg__Detection__FIELD_NAME__distance_m[] = "distance_m";

static rosidl_runtime_c__type_description__Field robot_nav_interfaces__msg__Detection__FIELDS[] = {
  {
    {robot_nav_interfaces__msg__Detection__FIELD_NAME__class_name, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_nav_interfaces__msg__Detection__FIELD_NAME__x_min, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_nav_interfaces__msg__Detection__FIELD_NAME__y_min, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_nav_interfaces__msg__Detection__FIELD_NAME__x_max, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_nav_interfaces__msg__Detection__FIELD_NAME__y_max, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_nav_interfaces__msg__Detection__FIELD_NAME__confidence, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_nav_interfaces__msg__Detection__FIELD_NAME__distance_m, 10, 10},
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
robot_nav_interfaces__msg__Detection__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_nav_interfaces__msg__Detection__TYPE_NAME, 34, 34},
      {robot_nav_interfaces__msg__Detection__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string class_name\n"
  "int32 x_min\n"
  "int32 y_min\n"
  "int32 x_max\n"
  "int32 y_max\n"
  "float32 confidence\n"
  "float32 distance_m";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_nav_interfaces__msg__Detection__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_nav_interfaces__msg__Detection__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 104, 104},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_nav_interfaces__msg__Detection__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_nav_interfaces__msg__Detection__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
