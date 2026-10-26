#pragma once

#include <cmath>

#include "robot_nav_controller/command.hpp"
#include "robot_nav_controller/pose.hpp"
#include "robot_nav_controller/utils.hpp"

// Advances a planar unicycle by dt under a constant command, integrating the
// circular arc exactly. Shared by the closed-loop test and the experiments so
// they simulate identical physics.
inline void step_unicycle(Pose& pose, const Command& cmd, double dt) {
    const double next = pose.theta + cmd.angular_velocity * dt;
    if (std::abs(cmd.angular_velocity) < 1e-12) {
        pose.x += cmd.linear_velocity * std::cos(pose.theta) * dt;
        pose.y += cmd.linear_velocity * std::sin(pose.theta) * dt;
    } else {
        const double radius = cmd.linear_velocity / cmd.angular_velocity;
        pose.x += radius * (std::sin(next) - std::sin(pose.theta));
        pose.y -= radius * (std::cos(next) - std::cos(pose.theta));
    }
    pose.theta = robot_nav_controller::wrap_angle(next);
}
