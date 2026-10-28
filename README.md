# Robot Navigation Sim

## Goal

Build a simulated robot that uses a camera feed to detect obstacles and decide whether to move or stop.

## System Blueprint

```text
Gazebo simulated camera
        ↓
perception_node (YOLO)
        ↓ /detections
planner_node (Python)        ← odometry, /driver/status
        ↓ /goal: the destination, or a temporary detour goal around an obstacle
driver_node (C++ Controller) ← odometry, depth image (safety stop)
        ↓ /model/vehicle/cmd_vel
Gazebo simulated robot motion
```

The planner decides *where* to go and the driver gets there safely. Run it with
`./scripts/run_reactive_demo.sh` (or `false` to disable detours). Before/after
numbers for this design are in `docs/goal-based-navigation-results.md`.

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
