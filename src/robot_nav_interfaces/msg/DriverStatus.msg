# What driver_node is doing, published every control tick on /driver/status.
# Values match robot_nav_controller::DriverStatus (driver_logic.hpp);
# driver_node static_asserts that they agree.
uint8 RUNNING=0        # Driving toward the goal.
uint8 GOAL_REACHED=1   # Within goal_tolerance of the goal; stopped.
uint8 INVALID_INPUT=2  # No odometry yet, or a non-finite pose/goal; stopped.
uint8 SAFETY_STOP=3    # Obstacle inside hard_stop_distance; turning only.
uint8 NO_GOAL=4        # No goal received yet; stopped.
uint8 DEPTH_STALE=5    # No recent depth image; turning only.

uint8 status

# The goal this status refers to (NaN before the first goal). Right after the
# planner switches goals, this tells a status about the old goal apart from
# one about the new goal.
float64 goal_x
float64 goal_y

float64 distance_to_goal_m  # NaN when there is no pose or goal.
