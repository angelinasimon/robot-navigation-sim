#include "robot_nav_controller/controller.hpp"

#include <algorithm>
#include <cmath>

#include "robot_nav_controller/utils.hpp"

Controller::Controller(double kp, double ka, double stop_threshold,
                       double spin_threshold, double max_linear_velocity,
                       double max_angular_velocity)
    : kp_(kp),
      ka_(ka),
      stop_threshold_(stop_threshold),
      spin_threshold_(spin_threshold),
      max_linear_velocity_(max_linear_velocity),
      max_angular_velocity_(max_angular_velocity) {}

Command Controller::compute_command(const Pose& current_pose,
                                    const Pose& goal_pose) const {
  const Command stop{0.0, 0.0};

  if (!std::isfinite(current_pose.x) || !std::isfinite(current_pose.y) ||
      !std::isfinite(current_pose.theta) || !std::isfinite(goal_pose.x) ||
      !std::isfinite(goal_pose.y)) {
    return stop;
  }

  // rho: distance to the goal.
  const double dx = goal_pose.x - current_pose.x;
  const double dy = goal_pose.y - current_pose.y;
  const double rho = std::hypot(dx, dy);

  // Close enough: stop. (alpha is meaningless right on top of the goal.)
  if (rho < stop_threshold_) {
    return stop;
  }

  // alpha: angle from the robot's nose to the goal, wrapped to (-pi, pi].
  const double alpha =
      robot_nav_controller::wrap_angle(std::atan2(dy, dx) - current_pose.theta);

  Command cmd;
  cmd.angular_velocity = ka_ * alpha;
  if (std::fabs(alpha) > spin_threshold_) {
    // Goal is too far off to the side or behind: turn in place first.
    cmd.linear_velocity = 0.0;
  } else {
    cmd.linear_velocity = kp_ * rho;
  }

  cmd.linear_velocity = std::clamp(cmd.linear_velocity, -max_linear_velocity_,
                                   max_linear_velocity_);
  cmd.angular_velocity = std::clamp(cmd.angular_velocity, -max_angular_velocity_,
                                    max_angular_velocity_);
  return cmd;
}