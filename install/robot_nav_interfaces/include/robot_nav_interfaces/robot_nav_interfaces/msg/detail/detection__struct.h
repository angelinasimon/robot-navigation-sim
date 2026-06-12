// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_nav_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_nav_interfaces/msg/detection.h"


#ifndef ROBOT_NAV_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_H_
#define ROBOT_NAV_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'class_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Detection in the package robot_nav_interfaces.
typedef struct robot_nav_interfaces__msg__Detection
{
  rosidl_runtime_c__String class_name;
  int32_t x_min;
  int32_t y_min;
  int32_t x_max;
  int32_t y_max;
  float confidence;
  float distance_m;
} robot_nav_interfaces__msg__Detection;

// Struct for a sequence of robot_nav_interfaces__msg__Detection.
typedef struct robot_nav_interfaces__msg__Detection__Sequence
{
  robot_nav_interfaces__msg__Detection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_nav_interfaces__msg__Detection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_NAV_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_H_
