# Robot Navigation Sim

## Goal

Build a simulated robot that uses a camera feed to detect obstacles and decide whether to move or stop.

## System Blueprint

```text
Gazebo simulated camera
        ↓
perception_node
        ↓ detections
planner_node
        ↓ stop/go command
robot_driver
        ↓ velocity command
Gazebo simulated robot motion# robot-navigation-sim
