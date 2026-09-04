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
        ↓ cruise / avoid let / avoid right command
robot_driver
        ↓ velocity command
Gazebo simulated robot motion# robot-navigation-sim
## Detection Data Contract

One detection should contain:

- object_class → what object was detected
- bounding_box → where the object appears in the image
- confidence → how sure the detector is
- estimated_distance → rough distance from robot


## Evaluation Plan

Eventually measure:

- success_rate → did robot reach the goal?
- collision_rate → did robot hit obstacle?
- time_to_goal → how long navigation took
- detection_latency → time from obstacle appearing to stop command
