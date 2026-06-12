// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from robot_nav_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice
#ifndef ROBOT_NAV_INTERFACES__MSG__DETAIL__DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define ROBOT_NAV_INTERFACES__MSG__DETAIL__DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "robot_nav_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "robot_nav_interfaces/msg/detail/detection__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_nav_interfaces
bool cdr_serialize_robot_nav_interfaces__msg__Detection(
  const robot_nav_interfaces__msg__Detection * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_nav_interfaces
bool cdr_deserialize_robot_nav_interfaces__msg__Detection(
  eprosima::fastcdr::Cdr &,
  robot_nav_interfaces__msg__Detection * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_nav_interfaces
size_t get_serialized_size_robot_nav_interfaces__msg__Detection(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_nav_interfaces
size_t max_serialized_size_robot_nav_interfaces__msg__Detection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_nav_interfaces
bool cdr_serialize_key_robot_nav_interfaces__msg__Detection(
  const robot_nav_interfaces__msg__Detection * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_nav_interfaces
size_t get_serialized_size_key_robot_nav_interfaces__msg__Detection(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_nav_interfaces
size_t max_serialized_size_key_robot_nav_interfaces__msg__Detection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_nav_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_nav_interfaces, msg, Detection)();

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_NAV_INTERFACES__MSG__DETAIL__DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
