# Standalone navigation controller

Build and run Day 5 without ROS or third-party test libraries:

```sh
cmake -S src/robot_nav_controller -B /tmp/robot-nav-controller \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON
cmake --build /tmp/robot-nav-controller
ctest --test-dir /tmp/robot-nav-controller --output-on-failure
```

The public interface is in `include/robot_nav_controller/`: `pose.hpp`,
`command.hpp`, `control_result.hpp`, `controller.hpp`, and `utils.hpp`.

```cpp
#include "robot_nav_controller/controller.hpp"

Controller controller;
ControlResult result = controller.compute_command(Pose{0, 0, 0}, Pose{2, 1, 0});
// result.command: velocities to send; result.status: why.
```

`compute_command` uses proportional distance and wrapped heading errors.
It spins in place when the heading error exceeds 0.5 radians, otherwise
moves forward while steering. Default gains are 1.0 (distance) and 2.0
(heading); speeds are capped at 0.5 m/s and 1.5 rad/s. At or within 0.05 m
of the goal both velocities are zero. Goal orientation is unconstrained;
`goal.theta` does not affect steering. Constructor parameters must be finite;
gains and velocity limits must be positive, the stop threshold nonnegative,
and the spin threshold greater than 0 and at most pi/2 so the robot never drives forward
with the goal behind it. Invalid parameters throw `std::invalid_argument`.

Each result carries a `ControllerStatus`, checked in this order:

- `INVALID_INPUT`: a pose field (including `goal.theta`) is non-finite.
  The command is zero.
- `GOAL_REACHED`: the robot is within the stop threshold. The command is zero.
- `RUNNING`: otherwise, including turning in place.

Use the status, not a zero command, to detect arrival. The controller is
stateless, so `GOAL_REACHED` becomes `RUNNING` again if the robot drifts out
of tolerance.

The unit test covers stopping, proportional motion, speed limits, turn
signs, angle wrapping, invalid input, status reporting, and parameter
validation. The closed-loop test integrates a planar unicycle at 50 Hz for 16 combinations
of starting poses and goals, checking speed bounds, `GOAL_REACHED` within 0.05 m,
and a stable stop within 120 simulated seconds. This is an obstacle-free
kinematic test; it does not model wheel dynamics or collisions.
