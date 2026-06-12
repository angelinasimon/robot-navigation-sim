// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_nav_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_nav_interfaces/msg/detection.hpp"


#ifndef ROBOT_NAV_INTERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_
#define ROBOT_NAV_INTERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_nav_interfaces/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_nav_interfaces
{

namespace msg
{

namespace builder
{

class Init_Detection_distance_m
{
public:
  explicit Init_Detection_distance_m(::robot_nav_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  ::robot_nav_interfaces::msg::Detection distance_m(::robot_nav_interfaces::msg::Detection::_distance_m_type arg)
  {
    msg_.distance_m = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_nav_interfaces::msg::Detection msg_;
};

class Init_Detection_confidence
{
public:
  explicit Init_Detection_confidence(::robot_nav_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_distance_m confidence(::robot_nav_interfaces::msg::Detection::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Detection_distance_m(msg_);
  }

private:
  ::robot_nav_interfaces::msg::Detection msg_;
};

class Init_Detection_y_max
{
public:
  explicit Init_Detection_y_max(::robot_nav_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_confidence y_max(::robot_nav_interfaces::msg::Detection::_y_max_type arg)
  {
    msg_.y_max = std::move(arg);
    return Init_Detection_confidence(msg_);
  }

private:
  ::robot_nav_interfaces::msg::Detection msg_;
};

class Init_Detection_x_max
{
public:
  explicit Init_Detection_x_max(::robot_nav_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_y_max x_max(::robot_nav_interfaces::msg::Detection::_x_max_type arg)
  {
    msg_.x_max = std::move(arg);
    return Init_Detection_y_max(msg_);
  }

private:
  ::robot_nav_interfaces::msg::Detection msg_;
};

class Init_Detection_y_min
{
public:
  explicit Init_Detection_y_min(::robot_nav_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_x_max y_min(::robot_nav_interfaces::msg::Detection::_y_min_type arg)
  {
    msg_.y_min = std::move(arg);
    return Init_Detection_x_max(msg_);
  }

private:
  ::robot_nav_interfaces::msg::Detection msg_;
};

class Init_Detection_x_min
{
public:
  explicit Init_Detection_x_min(::robot_nav_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_y_min x_min(::robot_nav_interfaces::msg::Detection::_x_min_type arg)
  {
    msg_.x_min = std::move(arg);
    return Init_Detection_y_min(msg_);
  }

private:
  ::robot_nav_interfaces::msg::Detection msg_;
};

class Init_Detection_class_name
{
public:
  Init_Detection_class_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Detection_x_min class_name(::robot_nav_interfaces::msg::Detection::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_Detection_x_min(msg_);
  }

private:
  ::robot_nav_interfaces::msg::Detection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_nav_interfaces::msg::Detection>()
{
  return robot_nav_interfaces::msg::builder::Init_Detection_class_name();
}

}  // namespace robot_nav_interfaces

#endif  // ROBOT_NAV_INTERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_
