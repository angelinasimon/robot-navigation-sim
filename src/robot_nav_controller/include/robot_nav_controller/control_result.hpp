#pragma once

#include "robot_nav_controller/command.hpp"

// Why the controller produced its command. Use this, not a zero command, to
// detect arrival: RUNNING can also carry a zero command in degenerate cases.
enum class ControllerStatus {
    RUNNING,        // Driving toward the goal (including turning in place).
    GOAL_REACHED,   // Within stop_threshold of the goal; command is zero.
    INVALID_INPUT,  // A pose field was non-finite; command is zero.
};

struct ControlResult {
    Command command;
    ControllerStatus status;
};
