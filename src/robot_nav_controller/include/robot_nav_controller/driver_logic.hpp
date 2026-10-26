#pragma once

#include <optional>

#include "robot_nav_controller/command.hpp"
#include "robot_nav_controller/controller.hpp"
#include "robot_nav_controller/pose.hpp"

namespace robot_nav_controller {

// What the driver is doing. Values match robot_nav_interfaces/msg/DriverStatus.
enum class DriverStatus {
    RUNNING = 0,        // Driving toward the goal.
    GOAL_REACHED = 1,   // Within goal_tolerance of the goal; command is zero.
    INVALID_INPUT = 2,  // No odometry yet, or a non-finite pose/goal; command is zero.
    SAFETY_STOP = 3,    // Obstacle inside hard_stop_distance: no forward motion,
                        // turning still allowed so the robot can face a new goal.
    NO_GOAL = 4,        // No goal received yet; command is zero.
    DEPTH_STALE = 5,    // Depth safety is on but no reading newer than depth_timeout:
                        // driving blind is unsafe, so no forward motion (turning allowed).
};

// Defaults are the limits robot_driver.py was tuned to for this vehicle.
struct DriverParams {
    double kp = 1.0;
    double ka = 0.55;
    double goal_tolerance = 0.3;
    double spin_threshold = 0.5;
    double max_linear_velocity = 0.35;
    double max_angular_velocity = 0.32;

    bool depth_safety_enabled = true;
    double hard_stop_distance = 0.85;  // Metres.
    double depth_timeout = 0.75;       // Seconds; older readings count as no reading.
};

struct Goal {
    double x, y;
};

struct DriverInputs {
    std::optional<Pose> pose;           // Latest odometry; empty until the first message.
    std::optional<Goal> goal;           // Latest goal from the planner.
    std::optional<double> front_depth;  // From front_depth(); empty until the first image.
    double depth_stamp = 0.0;           // When front_depth was measured, seconds.
};

struct DriverOutput {
    Command command;
    DriverStatus status;
    double distance_to_goal;  // NaN when there is no pose or goal.
};

// Decides the wheel command each control tick. Stateless: the output depends
// only on the inputs, so the same inputs always give the same command. Time is
// passed in rather than read from a clock, so tests can control it.
class DriverLogic {
 public:
    // Throws std::invalid_argument for invalid parameters, including any the
    // Controller rejects.
    explicit DriverLogic(const DriverParams& params = DriverParams{});

    DriverOutput update(const DriverInputs& inputs, double now_s) const;

 private:
    DriverParams params_;
    Controller controller_;
};

}  // namespace robot_nav_controller
