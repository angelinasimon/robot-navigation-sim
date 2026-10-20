#pragma once

#include "robot_nav_controller/command.hpp"
#include "robot_nav_controller/pose.hpp"

// Position-only goal controller. Distances are metres and angles are radians.
// goal_pose.theta is intentionally ignored; final orientation is unconstrained.
class Controller {
 public:
    // Throws std::invalid_argument unless every parameter is finite, the gains
    // and velocity limits are > 0, and the thresholds are >= 0. A
    // spin_threshold >= pi never triggers turn-in-place.
    Controller(double kp = 1.0, double ka = 2.0,
               double stop_threshold = 0.05, double spin_threshold = 0.5,
               double max_linear_velocity = 0.5,
               double max_angular_velocity = 1.5);

    // Non-finite poses produce a zero command.
    Command compute_command(const Pose& current_pose, const Pose& goal_pose) const;

 private:
    double kp_, ka_, stop_threshold_, spin_threshold_;
    double max_linear_velocity_, max_angular_velocity_;
};
