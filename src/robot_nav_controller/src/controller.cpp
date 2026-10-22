#include "robot_nav_controller/controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "robot_nav_controller/utils.hpp"

namespace {

double require_positive(const char* name, double value) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and > 0, got " +
                                std::to_string(value));
  }
  return value;
}

double require_nonnegative(const char* name, double value) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and >= 0, got " +
                                std::to_string(value));
  }
  return value;
}

// Above pi/2 the robot would drive forward with the goal behind it, moving
// away from the goal while it turns. At 0 it could only drive with exactly
// zero heading error, which it never reaches, so it would turn forever.
double require_spin_threshold(double value) {
  require_positive("spin_threshold", value);
  if (value > robot_nav_controller::kPi / 2.0) {
    throw std::invalid_argument("spin_threshold must be <= pi/2, got " +
                                std::to_string(value));
  }
  return value;
}

}  // namespace

Controller::Controller(double kp, double ka, double stop_threshold,
                       double spin_threshold, double max_linear_velocity,
                       double max_angular_velocity)
    : kp_(require_positive("kp", kp)),
      ka_(require_positive("ka", ka)),
      stop_threshold_(require_nonnegative("stop_threshold", stop_threshold)),
      spin_threshold_(require_spin_threshold(spin_threshold)),
      max_linear_velocity_(
          require_positive("max_linear_velocity", max_linear_velocity)),
      max_angular_velocity_(
          require_positive("max_angular_velocity", max_angular_velocity)) {}

ControlResult Controller::compute_command(const Pose& current_pose,
                                          const Pose& goal_pose) const {
  const Command stop{0.0, 0.0};

  if (!std::isfinite(current_pose.x) || !std::isfinite(current_pose.y) ||
      !std::isfinite(current_pose.theta) || !std::isfinite(goal_pose.x) ||
      !std::isfinite(goal_pose.y) || !std::isfinite(goal_pose.theta)) {
    return {stop, ControllerStatus::INVALID_INPUT};
  }

  // rho: distance to the goal.
  const double dx = goal_pose.x - current_pose.x;
  const double dy = goal_pose.y - current_pose.y;
  const double rho = std::hypot(dx, dy);

  // Close enough: stop. (alpha is meaningless right on top of the goal.)
  if (rho <= stop_threshold_) {
    return {stop, ControllerStatus::GOAL_REACHED};
  }

  // alpha: angle from the robot's nose to the goal, wrapped to (-pi, pi].
  const double alpha =
      robot_nav_controller::wrap_angle(std::atan2(dy, dx) - current_pose.theta);

  Command cmd;
  cmd.angular_velocity = ka_ * alpha;
  // Forward speed tapers smoothly from full (facing the goal) to zero at
  // |alpha| = spin_threshold, and stays zero beyond it (turn in place). Using
  // cos keeps the taper flat near alpha = 0, so small heading noise while
  // driving straight barely changes the speed. The denominator is positive
  // because the constructor guarantees spin_threshold is in (0, pi/2].
  // Cap the speed before tapering: tapering first and capping after would let
  // the cap flatten most of the taper whenever kp * rho exceeds the limit.
  const double cos_spin = std::cos(spin_threshold_);
  const double scale =
      std::clamp((std::cos(alpha) - cos_spin) / (1.0 - cos_spin), 0.0, 1.0);
  cmd.linear_velocity = std::min(kp_ * rho, max_linear_velocity_) * scale;

  cmd.angular_velocity = std::clamp(cmd.angular_velocity, -max_angular_velocity_,
                                    max_angular_velocity_);
  return {cmd, ControllerStatus::RUNNING};
}