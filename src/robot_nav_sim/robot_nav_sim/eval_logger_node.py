import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from robot_nav_interfaces.msg import Detection


class EvalLogger(Node):
    def __init__(self):
        super().__init__('eval_logger_node')

        self.declare_parameter('trial_id', 'trial_001')
        self.declare_parameter('scenario_type', 'manual')
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('obstacle_x', 2.5)
        self.declare_parameter('obstacle_y', 0.0)
        self.declare_parameter('timeout_s', 3000.0)

        self.trial_id = self.get_parameter('trial_id').value
        self.scenario_type = self.get_parameter('scenario_type').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.obstacle_x = self.get_parameter('obstacle_x').value
        self.obstacle_y = self.get_parameter('obstacle_y').value
        self.timeout_s = self.get_parameter('timeout_s').value

        self.start_time = time.time()
        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.current_x = None
        self.current_y = None

        self.detected_obstacle = False
        self.first_detection_time_s = None
        self.min_distance_to_obstacle = float('inf')
        self.done = False

        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/vehicle/odometry',
            self.odom_callback,
            10
        )

        self.det_sub = self.create_subscription(
            Detection,
            'detections',
            self.detection_callback,
            10
        )

        self.timer = self.create_timer(0.2, self.check_trial)

        self.output_path = (
            Path.home() /
            "robot-navigation-sim" /
            "data" /
            "eval_trials.csv"
        )
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        self.get_logger().info(
            f'Eval logger started for {self.trial_id}. '
            f'Goal=({self.goal_x}, {self.goal_y}), '
            f'obstacle=({self.obstacle_x}, {self.obstacle_y})'
        )

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.start_yaw = 0.0

        self.current_x = x
        self.current_y = y

        dist_to_obstacle = math.hypot(x - self.obstacle_x, y - self.obstacle_y)
        self.min_distance_to_obstacle = min(
            self.min_distance_to_obstacle,
            dist_to_obstacle
        )

    def detection_callback(self, msg):
        if not self.detected_obstacle:
            self.detected_obstacle = True
            self.first_detection_time_s = time.time() - self.start_time

    def check_trial(self):
        if self.done or self.current_x is None:
            return

        elapsed = time.time() - self.start_time
        dist_to_goal = math.hypot(
            self.current_x - self.goal_x,
            self.current_y - self.goal_y
        )

        if dist_to_goal < 0.3:
            self.write_row(success=True, timeout=False, time_to_goal_s=elapsed)
            self.done = True
            self.get_logger().info('Trial success. Logged row.')

        elif elapsed > self.timeout_s:
            self.write_row(success=False, timeout=True, time_to_goal_s=elapsed)
            self.done = True
            self.get_logger().info('Trial timeout. Logged row.')

    def write_row(self, success, timeout, time_to_goal_s):
        file_exists = self.output_path.exists()

        with self.output_path.open('a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=[
                'trial_id',
                'scenario_type',
                'start_x',
                'start_y',
                'start_yaw',
                'goal_x',
                'goal_y',
                'obstacle_x',
                'obstacle_y',
                'detected_obstacle',
                'first_detection_time_s',
                'min_distance_to_obstacle_m',
                'success',
                'collision',
                'timeout',
                'time_to_goal_s',
                'notes',
            ])

            if not file_exists:
                writer.writeheader()

            writer.writerow({
                'trial_id': self.trial_id,
                'scenario_type': self.scenario_type,
                'start_x': self.start_x,
                'start_y': self.start_y,
                'start_yaw': self.start_yaw,
                'goal_x': self.goal_x,
                'goal_y': self.goal_y,
                'obstacle_x': self.obstacle_x,
                'obstacle_y': self.obstacle_y,
                'detected_obstacle': self.detected_obstacle,
                'first_detection_time_s': self.first_detection_time_s,
                'min_distance_to_obstacle_m': self.min_distance_to_obstacle,
                'success': success,
                'collision': '',
                'timeout': timeout,
                'time_to_goal_s': time_to_goal_s,
                'notes': '',
            })


def main(args=None):
    rclpy.init(args=args)
    node = EvalLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
