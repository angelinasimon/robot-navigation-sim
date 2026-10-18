# Standalone navigation controller

Build and run Day 5 without ROS or third-party test libraries:

```sh
cmake -S src/robot_nav_controller -B /tmp/robot-nav-controller \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON
cmake --build /tmp/robot-nav-controller
ctest --test-dir /tmp/robot-nav-controller --output-on-failure
```

The public interface is in `include/robot_nav_controller/`: `pose.hpp`,
`command.hpp`, `controller.hpp`, and `utils.hpp`.

```cpp
#include "robot_nav_controller/controller.hpp"

Controller controller;
Command command = controller.compute_command(Pose{0, 0, 0}, Pose{2, 1, 0});
```

`compute_command` uses proportional distance and wrapped heading errors.
It spins in place when the heading error exceeds 0.5 radians, otherwise
moves forward while steering. Default gains are 1.0 (distance) and 2.0
(heading); speeds are capped at 0.5 m/s and 1.5 rad/s. At or within 0.05 m
of the goal both velocities are zero. Goal orientation is unconstrained;
`goal.theta` does not affect steering. All pose fields must be finite.
Invalid poses return a zero command. Constructor parameters must be finite
and positive, with the spin threshold at most pi/2; invalid parameters throw
`std::invalid_argument`.

The unit test covers stopping, proportional motion, speed limits, turn
signs, angle wrapping, invalid input, and parameter validation. The
closed-loop test integrates a planar unicycle at 50 Hz for 16 combinations
of starting poses and goals, checking speed bounds, arrival within 0.05 m,
and a stable stop within 120 simulated seconds. This is an obstacle-free
kinematic test; it does not model wheel dynamics or collisions.
