"""
Choose the goal the robot drives to and publish it on /goal for driver_node.

Normally the goal is the destination (goal_x, goal_y). When perception sees an
obstacle blocking the straight path, the planner switches to a temporary goal
beside it, then back to the destination once the driver reaches it. The
decisions live in goal_planner.py and detour.py; this node only converts
messages and timestamps.
"""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from robot_nav_interfaces.msg import Detection, DriverStatus
from robot_nav_sim.detour import obstacle_in_robot_frame, robot_to_world
from robot_nav_sim.goal_planner import GoalPlanner, PlannerParams


def quaternion_to_yaw(q):
    """Extract yaw rotation about Z from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PlannerNode(Node):
    """Publishes the active goal; switches to detour goals around obstacles."""

    def __init__(self):
        super().__init__('planner_node')

        params = PlannerParams(
            destination=(self.declare_parameter('goal_x', 5.0).value,
                         self.declare_parameter('goal_y', 0.0).value),
            avoidance_enabled=self.declare_parameter('avoidance_enabled', True).value,
            pass_distance=self.declare_parameter('pass_distance', 1.5).value,
            past_distance=self.declare_parameter('past_distance', 0.5).value,
            detour_timeout=self.declare_parameter('detour_timeout', 20.0).value,
        )
        self.planner = GoalPlanner(params)

        # Which detections count as obstacles.
        self.trigger_distance = self.declare_parameter('trigger_distance', 2.0).value
        self.min_confidence = self.declare_parameter('min_confidence', 0.35).value

        # Camera geometry, to place a detection in the world. These must match
        # the rgbd_camera in worlds/reactive_avoidance_demo.sdf: 320 px wide,
        # 1.047 rad field of view, mounted 0.55 m in front of the vehicle origin.
        # (The old planner had image_width=640 here, which silently flipped
        # left/right for half of all detections.)
        self.image_width = self.declare_parameter('image_width', 320).value
        self.horizontal_fov = self.declare_parameter('horizontal_fov', 1.047).value
        self.camera_forward_offset = self.declare_parameter(
            'camera_forward_offset', 0.55).value

        self.goal_frame = self.declare_parameter('goal_frame', 'vehicle/odom').value

        # Perception publishes a heartbeat every frame, even with nothing seen,
        # so silence means it has died. The planner keeps navigating (the
        # driver's depth stop still protects against obstacles) but says so.
        self.detection_timeout = self.declare_parameter('detection_timeout', 1.0).value
        self.last_detection_time = self.now_s()

        self.robot_pose = None  # (x, y, yaw) from odometry; None until the first message.

        self.create_subscription(Detection, 'detections', self.on_detection, 10)
        self.create_subscription(Odometry, '/model/vehicle/odometry', self.on_odometry, 10)
        self.create_subscription(DriverStatus, '/driver/status', self.on_driver_status, 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal', 10)

        # Republish the goal at 2 Hz even when it doesn't change, so a driver
        # that starts (or restarts) after the planner still receives it.
        self.create_timer(0.5, self.on_timer)

        self.get_logger().info(
            f'Planner started: destination=({params.destination[0]:.2f}, '
            f'{params.destination[1]:.2f}), avoidance_enabled={params.avoidance_enabled}')
        self.publish_goal()

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def report(self, change):
        """Log a change reported by GoalPlanner and publish the new goal at once."""
        if change is not None:
            self.get_logger().info(f'[{self.planner.mode.value}] {change}')
            self.publish_goal()

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.goal_frame
        msg.pose.position.x = float(self.planner.goal[0])
        msg.pose.position.y = float(self.planner.goal[1])
        msg.pose.orientation.w = 1.0  # The driver ignores the goal heading.
        self.goal_publisher.publish(msg)

    def on_timer(self):
        self.report(self.planner.on_tick(self.now_s()))
        self.publish_goal()

        silence = self.now_s() - self.last_detection_time
        if silence > self.detection_timeout:
            self.get_logger().warn(
                f'No detections (not even heartbeats) for {silence:.1f} s: '
                f'perception may be down. Obstacles will not be avoided.',
                throttle_duration_sec=5.0)

    def on_odometry(self, msg):
        p = msg.pose.pose
        self.robot_pose = (p.position.x, p.position.y, quaternion_to_yaw(p.orientation))

    def on_driver_status(self, msg):
        if msg.status == DriverStatus.GOAL_REACHED:
            self.report(self.planner.on_goal_reached((msg.goal_x, msg.goal_y)))

    def on_detection(self, detection):
        self.last_detection_time = self.now_s()

        if detection.confidence < self.min_confidence:
            return  # Includes perception's empty heartbeat messages.
        # Perception sends distance_m = -1.0 when the depth at the box is
        # missing: not enough information to place the obstacle.
        if detection.distance_m <= 0.0:
            self.get_logger().warn(
                f'Detected {detection.class_name} (confidence {detection.confidence:.2f}) '
                f'but no valid depth; ignoring this frame.',
                throttle_duration_sec=2.0)
            return
        if detection.distance_m >= self.trigger_distance:
            return
        if self.robot_pose is None:
            self.get_logger().warn('Obstacle seen but no odometry yet; cannot place it.',
                                   throttle_duration_sec=2.0)
            return

        bbox_center_x = (detection.x_min + detection.x_max) / 2.0
        in_robot_frame = obstacle_in_robot_frame(
            bbox_center_x, self.image_width, self.horizontal_fov, detection.distance_m,
            self.camera_forward_offset)
        obstacle = robot_to_world(self.robot_pose, in_robot_frame)
        self.report(self.planner.on_obstacle(self.robot_pose[:2], obstacle, self.now_s()))


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
