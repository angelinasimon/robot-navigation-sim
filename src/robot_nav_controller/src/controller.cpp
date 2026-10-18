#include "robot_nav_controller/controller.hpp"
#include "robot_nav_controller/utils.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

Controller::Controller(double kp, double ka, double stop_threshold,
                       double spin_threshold, double max_linear_velocity,
                       double max_angular_velocity)
    : kp_(kp), ka_(ka), stop_threshold_(stop_threshold),
      spin_threshold_(spin_threshold), max_linear_velocity_(max_linear_velocity),
      max_angular_velocity_(max_angular_velocity) {
    for (double value : {kp, ka, stop_threshold, spin_threshold,
                         max_linear_velocity, max_angular_velocity}) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument("Controller parameters must be finite and positive");
        }
    }
    if (spin_threshold > robot_nav_controller::kPi / 2.0) {
        throw std::invalid_argument("spin_threshold must not exceed pi/2");
    }
}

Command Controller::compute_command(const Pose& current, const Pose& goal) const {
    for (double value : {current.x, current.y, current.theta,
                         goal.x, goal.y, goal.theta}) {
        if (!std::isfinite(value)) return {0.0, 0.0};
    }
    const double dx = goal.x - current.x;
    const double dy = goal.y - current.y;
    const double distance = std::hypot(dx, dy);
    if (!std::isfinite(distance) || distance <= stop_threshold_) return {0.0, 0.0};

    const double error = robot_nav_controller::wrap_angle(
        std::atan2(dy, dx) - robot_nav_controller::wrap_angle(current.theta));
    const double angular = std::clamp(ka_ * error,
                                     -max_angular_velocity_, max_angular_velocity_);
    const double linear = std::abs(error) > spin_threshold_
        ? 0.0 : std::min(kp_ * distance, max_linear_velocity_);
    return {linear, angular};
}
