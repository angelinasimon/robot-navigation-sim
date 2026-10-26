#include "robot_nav_controller/driver_logic.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace robot_nav_controller {

namespace {

const DriverParams& validated(const DriverParams& p) {
    if (!std::isfinite(p.hard_stop_distance) || p.hard_stop_distance < 0.0) {
        throw std::invalid_argument("hard_stop_distance must be finite and >= 0, got " +
                                    std::to_string(p.hard_stop_distance));
    }
    if (!std::isfinite(p.depth_timeout) || p.depth_timeout <= 0.0) {
        throw std::invalid_argument("depth_timeout must be finite and > 0, got " +
                                    std::to_string(p.depth_timeout));
    }
    return p;
}

DriverStatus from_controller(ControllerStatus s) {
    switch (s) {
        case ControllerStatus::RUNNING: return DriverStatus::RUNNING;
        case ControllerStatus::GOAL_REACHED: return DriverStatus::GOAL_REACHED;
        case ControllerStatus::INVALID_INPUT: return DriverStatus::INVALID_INPUT;
    }
    return DriverStatus::INVALID_INPUT;  // Unreachable; keeps compilers quiet.
}

}  // namespace

DriverLogic::DriverLogic(const DriverParams& params)
    : params_(validated(params)),
      controller_(params.kp, params.ka, params.goal_tolerance, params.spin_threshold,
                  params.max_linear_velocity, params.max_angular_velocity) {}

DriverOutput DriverLogic::update(const DriverInputs& in, double now_s) const {
    const Command stop{0.0, 0.0};
    const double unknown = std::numeric_limits<double>::quiet_NaN();

    if (!in.goal) return {stop, DriverStatus::NO_GOAL, unknown};
    if (!in.pose) return {stop, DriverStatus::INVALID_INPUT, unknown};

    const Pose goal_pose{in.goal->x, in.goal->y, 0.0};  // Goal heading is ignored.
    const double distance = std::hypot(goal_pose.x - in.pose->x, goal_pose.y - in.pose->y);
    const ControlResult res = controller_.compute_command(*in.pose, goal_pose);
    DriverOutput out{res.command, from_controller(res.status), distance};

    // Depth safety only matters while driving. It is the only protection against
    // obstacles the planner never detected, so it fails closed: stale or missing
    // depth stops forward motion (DEPTH_STALE), unlike robot_driver.py, which
    // ignored it and drove blind. An empty corridor arrives as front_depth == 0.0
    // and stops the robot too. Turning stays allowed in both cases so the robot
    // can still face a new goal.
    if (out.status != DriverStatus::RUNNING || !params_.depth_safety_enabled) return out;
    const bool depth_fresh =
        in.front_depth && (now_s - in.depth_stamp) < params_.depth_timeout;
    if (!depth_fresh) {
        out.command.linear_velocity = 0.0;
        out.status = DriverStatus::DEPTH_STALE;
    } else if (*in.front_depth < params_.hard_stop_distance) {
        out.command.linear_velocity = 0.0;
        out.status = DriverStatus::SAFETY_STOP;
    }
    return out;
}

}  // namespace robot_nav_controller
