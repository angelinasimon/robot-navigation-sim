#!/usr/bin/env bash
set -e

cd ~/robot-navigation-sim

# Usage:
#   ./scripts/run_reactive_demo.sh            -> avoidance ON (default)
#   ./scripts/run_reactive_demo.sh false       -> avoidance OFF (stop-only baseline)
AVOIDANCE_ENABLED="${1:-true}"

echo "Killing old processes..."
pkill -f gz || true
pkill -f ros2 || true
pkill -f perception_node || true
pkill -f planner_node || true
pkill -f robot_driver || true
pkill -f ros_gz_bridge || true
pkill -f parameter_bridge || true

echo "Sourcing ROS..."
source /opt/ros/jazzy/setup.bash

echo "Building..."
colcon build
source install/setup.bash

echo "Starting Gazebo..."
gz sim ~/robot-navigation-sim/worlds/reactive_avoidance_demo.sdf > logs/gazebo.log 2>&1 &
sleep 5

echo "Starting bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  "/model/vehicle/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" \
  "/model/vehicle/enable@std_msgs/msg/Bool]gz.msgs.Boolean" \
  "/model/vehicle/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
  "/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image" \
  > logs/bridge.log 2>&1 &
sleep 3

echo "Starting perception..."
ros2 run robot_nav_sim perception_node > logs/perception.log 2>&1 &
sleep 2

echo "Starting planner..."
ros2 run robot_nav_sim planner_node > logs/planner.log 2>&1 &
sleep 2

echo "Starting driver (avoidance_enabled=${AVOIDANCE_ENABLED})..."
ros2 run robot_nav_sim robot_driver --ros-args -p avoidance_enabled:="${AVOIDANCE_ENABLED}" > logs/driver.log 2>&1 &

echo ""
echo "Full loop is running. avoidance_enabled=${AVOIDANCE_ENABLED}"
echo "Press Play in Gazebo if needed."
echo ""
echo "Useful logs:"
echo "  tail -f logs/perception.log"
echo "  tail -f logs/planner.log"
echo "  tail -f logs/driver.log"
echo ""
echo "Useful checks:"
echo "  ros2 topic echo /cmd"
echo "  ros2 topic echo /detections"
echo ""
echo "To stop everything:"
echo "  pkill -f gz; pkill -f ros2"