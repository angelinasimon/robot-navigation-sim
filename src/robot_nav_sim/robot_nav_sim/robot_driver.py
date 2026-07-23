import math

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


def quaternion_to_yaw(q):
    """Extract yaw rotation about Z from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')

        # ---------------- Parameters ----------------
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('avoidance_enabled', True)
        self.declare_parameter('drive_without_odom', True)

        # Depth safety
        self.declare_parameter('depth_safety_enabled', True)
        self.declare_parameter('depth_topic', '/rgbd_camera/depth_image')
        self.declare_parameter('horizontal_fov', 1.047)

        # Vehicle footprint.
        # This is intentionally conservative because wheels get stuck.
        self.declare_parameter('wheel_separation', 1.0)
        self.declare_parameter('wheel_width', 0.1)

        # Important fix:
        # 0.25 was too tight. This protects wheel edges better.
        self.declare_parameter('side_safety_margin', 0.45)

        # Important fix:
        # These are larger because the vehicle has a physical body + wheels.
        self.declare_parameter('hard_stop_distance', 0.85)
        self.declare_parameter('avoid_distance', 1.75)
        self.declare_parameter('clear_distance', 1.95)

        self.declare_parameter('min_valid_depth', 0.05)
        self.declare_parameter('max_valid_depth', 5.0)

        # Important fix (root cause of "gets stuck"):
        # CREEP used to run for a fixed ~0.8s at ~0.07 m/s -- about 5-6 cm of
        # actual travel. That's nowhere near enough to clear an obstacle before
        # goal-seeking resumed, so the robot would swing back toward the goal,
        # re-enter the same depth safety corridor, and repeat
        # BRAKE->TURN->CREEP forever without ever making net progress.
        # Now CREEP (and REVERSE) are gated on actual distance traveled
        # (via odometry), with a time cap purely as a safety net.
        self.declare_parameter('creep_clearance_distance', 1.1)
        self.declare_parameter('creep_max_duration', 7.0)
        self.declare_parameter('creep_fallback_duration', 5.0)

        self.declare_parameter('reverse_min_distance', 0.4)
        self.declare_parameter('reverse_max_duration', 2.2)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.avoidance_enabled = self.get_parameter('avoidance_enabled').value
        self.drive_without_odom = self.get_parameter('drive_without_odom').value

        self.depth_safety_enabled = self.get_parameter('depth_safety_enabled').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.horizontal_fov = float(self.get_parameter('horizontal_fov').value)

        wheel_separation = float(self.get_parameter('wheel_separation').value)
        wheel_width = float(self.get_parameter('wheel_width').value)
        side_safety_margin = float(self.get_parameter('side_safety_margin').value)

        # Protected half-width of robot footprint.
        # Example:
        # wheel_separation=1.0, wheel_width=0.1, margin=0.45
        # inflated_half_width = 1.0 m
        # Full protected width = 2.0 m
        self.inflated_half_width = 0.5 * (wheel_separation + wheel_width) + side_safety_margin

        self.hard_stop_distance = float(self.get_parameter('hard_stop_distance').value)
        self.avoid_distance = float(self.get_parameter('avoid_distance').value)
        self.clear_distance = float(self.get_parameter('clear_distance').value)

        self.min_valid_depth = float(self.get_parameter('min_valid_depth').value)
        self.max_valid_depth = float(self.get_parameter('max_valid_depth').value)

        self.creep_clearance_distance = float(self.get_parameter('creep_clearance_distance').value)
        self.creep_max_duration = float(self.get_parameter('creep_max_duration').value)
        self.creep_fallback_duration = float(self.get_parameter('creep_fallback_duration').value)

        self.reverse_min_distance = float(self.get_parameter('reverse_min_distance').value)
        self.reverse_max_duration = float(self.get_parameter('reverse_max_duration').value)

        # Important fix:
        # CREEP used to drive in a dead-straight line once TURN finished.
        # If TURN only just barely cleared the front-center reading, a
        # straight-line creep can still clip the obstacle on the side --
        # the forward depth camera can't see flank clearance at all once the
        # obstacle is beside the robot. Keep gently arcing away from the
        # obstacle direction throughout the creep so lateral clearance keeps
        # growing the whole time, instead of being locked in by whatever the
        # initial turn happened to achieve.
        self.declare_parameter('creep_turn_bias', 0.12)
        self.creep_turn_bias = float(self.get_parameter('creep_turn_bias').value)

        # ---------------- ROS interfaces ----------------
        self.cmd_subscription = self.create_subscription(
            String,
            'cmd',
            self.listener_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/model/vehicle/odometry',
            self.odometry_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        self.velocity_publisher = self.create_publisher(
            Twist,
            '/model/vehicle/cmd_vel',
            10
        )

        self.enable_publisher = self.create_publisher(
            Bool,
            '/model/vehicle/enable',
            10
        )

        # ---------------- Motion tuning ----------------
        # Important fix (goal-seeking was slow):
        # forward_speed, heading_kp, and max_angular_speed were all tuned very
        # conservatively (like a real robot you don't want to break). For a
        # sim demo, that just means it wanders toward the goal slowly and
        # corrects heading sluggishly. Sped these up -- safety-relevant
        # distances (hard_stop/avoid/clear/inflated_half_width) are untouched.
        self.forward_speed = 0.35
        self.creep_speed = 0.25
        self.reverse_speed = -0.25
        self.turn_speed = 0.8

        self.heading_kp = 0.55
        self.max_angular_speed = 0.32

        self.brake_duration = 0.15

        # Important fix:
        # Instead of turning for one fixed duration only, we turn at least this long,
        # then keep turning until the depth corridor is clearer or max_turn_duration hits.
        self.min_turn_duration = 0.70
        self.max_turn_duration = 2.00

        # ---------------- State machine ----------------
        # States: DRIVING, AVOIDING, STOPPED, ARRIVED
        self.state = 'DRIVING'

        # Avoid phases: BRAKE, REVERSE, TURN, CREEP
        self.avoid_phase = None
        self.avoid_phase_start = None
        self.avoid_turn_direction = 1.0
        self.emergency_reverse = False

        # Used to self-correct the turn direction if it turns out to be
        # backwards for this robot's actual cmd_vel/frame convention (see
        # TURN phase handling below).
        self.turn_entry_front_depth = math.inf
        self.turn_direction_checked = False

        # Consecutive control ticks where the corridor has read "clear
        # enough" during TURN -- require several in a row before trusting it,
        # since one noisy good frame isn't proof the path is actually clear.
        self.turn_clear_streak = 0

        # Position at the start of the current avoid_phase (REVERSE/CREEP),
        # used to measure actual distance traveled during that phase.
        self.phase_start_x = None
        self.phase_start_y = None

        # Start with CRUISE so the robot can move before YOLO publishes.
        self.latest_command = 'CRUISE'
        self.last_command_time = self.get_clock().now()

        # Current pose from odometry.
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.has_warned_no_odom = False

        # Depth safety state.
        self.have_depth = False
        self.front_min_depth = math.inf
        self.left_min_depth = math.inf
        self.right_min_depth = math.inf
        self.last_depth_time = None
        self.unsupported_depth_warning_printed = False

        # Publish continuously.
        self.control_timer = self.create_timer(0.1, self.publish_control)

        self.get_logger().info(
            f'Robot driver started. Goal=({self.goal_x}, {self.goal_y}), '
            f'tolerance={self.goal_tolerance}m, avoidance_enabled={self.avoidance_enabled}, '
            f'depth_safety_enabled={self.depth_safety_enabled}, depth_topic={self.depth_topic}, '
            f'inflated_half_width={self.inflated_half_width:.2f}m, '
            f'hard_stop={self.hard_stop_distance:.2f}m, avoid={self.avoid_distance:.2f}m, '
            f'clear={self.clear_distance:.2f}m, '
            f'creep_clearance_distance={self.creep_clearance_distance:.2f}m, '
            f'reverse_min_distance={self.reverse_min_distance:.2f}m.'
        )

    # ---------------- Callbacks ----------------

    def odometry_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def listener_callback(self, msg):
        command = msg.data.strip().upper()

        if command == 'GO':
            command = 'CRUISE'

        self.latest_command = command
        self.last_command_time = self.get_clock().now()

        self.get_logger().info(f'Received YOLO/planner command: {self.latest_command}')

        if self.state == 'DRIVING' and self.latest_command in ('AVOID_LEFT', 'AVOID_RIGHT', 'STOP'):
            self.start_avoidance(self.latest_command)

    def depth_callback(self, msg):
        depth = self.image_to_depth_array(msg)

        if depth is None:
            self.have_depth = False
            return

        height, width = depth.shape

        # Important fix:
        # Use lower image area more aggressively.
        # Wheel-stuck problems are usually low/frontal, not high in the image.
        row_start = int(height * 0.45)
        row_end = int(height * 0.98)
        depth_crop = depth[row_start:row_end, :]

        cols = np.arange(width)
        center = (width - 1) / 2.0

        # Angle for each column.
        angles = ((cols - center) / center) * (self.horizontal_fov / 2.0)

        # Depth images usually give forward optical-axis distance.
        # Approx lateral offset from camera centerline:
        # lateral = forward_depth * tan(horizontal_angle)
        lateral_offsets = depth_crop * np.tan(angles)[None, :]

        valid = np.isfinite(depth_crop)
        valid &= depth_crop > self.min_valid_depth
        valid &= depth_crop < self.max_valid_depth

        # Main footprint corridor.
        in_robot_corridor = np.abs(lateral_offsets) <= self.inflated_half_width
        front_mask = valid & in_robot_corridor

        # Split by image side for choosing which way to turn.
        left_columns = cols < center
        right_columns = cols >= center

        left_mask = front_mask & left_columns[None, :]
        right_mask = front_mask & right_columns[None, :]

        self.front_min_depth = self.safe_depth_percentile(depth_crop[front_mask])
        self.left_min_depth = self.safe_depth_percentile(depth_crop[left_mask])
        self.right_min_depth = self.safe_depth_percentile(depth_crop[right_mask])

        self.have_depth = True
        self.last_depth_time = self.get_clock().now()

    # ---------------- Depth helpers ----------------

    def image_to_depth_array(self, msg):
        try:
            if msg.encoding == '32FC1':
                depth = np.frombuffer(msg.data, dtype=np.float32)
                depth = depth.reshape((msg.height, msg.width))
                return depth

            if msg.encoding == '16UC1':
                depth_mm = np.frombuffer(msg.data, dtype=np.uint16)
                depth_mm = depth_mm.reshape((msg.height, msg.width))
                return depth_mm.astype(np.float32) / 1000.0

            if not self.unsupported_depth_warning_printed:
                self.get_logger().warn(
                    f'Unsupported depth encoding: {msg.encoding}. Expected 32FC1 or 16UC1.'
                )
                self.unsupported_depth_warning_printed = True

            return None

        except Exception as e:
            self.get_logger().warn(f'Failed to parse depth image: {e}')
            return None

    def safe_depth_percentile(self, values):
        # Important fix (this was letting the robot drive straight into
        # contact -- the real bug behind the "goes right up to the person"
        # screenshot):
        # This used to return math.inf when there were zero valid pixels in
        # the corridor, which the safety logic reads as "infinitely far away,
        # totally clear." But an empty corridor almost always means the
        # opposite: the obstacle is so close that every pixel fell outside
        # the depth camera's sensing range (real and simulated RGBD sensors
        # commonly can't return a valid reading inside ~0.3-0.5m). Missing
        # data in a safety-critical zone must fail closed (assume something
        # is right on top of the sensor), not fail open (assume empty space).
        if values.size == 0:
            return 0.0

        # 5th percentile = near-ish obstacle without letting one noisy pixel dominate.
        return float(np.percentile(values, 5))

    def depth_is_fresh(self):
        if self.last_depth_time is None:
            return False

        age = (self.get_clock().now() - self.last_depth_time).nanoseconds / 1e9
        return age < 0.75

    def choose_depth_avoid_direction(self):
        """
        If left side is more blocked, turn right.
        If right side is more blocked, turn left.
        """
        if self.left_min_depth < self.right_min_depth:
            return 'AVOID_RIGHT'
        else:
            return 'AVOID_LEFT'

    def get_depth_safety_command(self):
        if not self.depth_safety_enabled:
            return None

        if not self.have_depth or not self.depth_is_fresh():
            return None

        if self.front_min_depth < self.hard_stop_distance:
            return 'STOP'

        if self.front_min_depth < self.avoid_distance:
            return self.choose_depth_avoid_direction()

        return None

    def depth_corridor_clear_enough(self):
        """
        Used before allowing the creep phase.
        This is stricter than just avoiding immediate collision.
        """
        if not self.depth_safety_enabled:
            return True

        if not self.have_depth or not self.depth_is_fresh():
            # Do not block forever if depth is missing.
            return True

        return self.front_min_depth >= self.clear_distance

    # ---------------- Motion helpers ----------------

    def seconds_since(self, start_time):
        return (self.get_clock().now() - start_time).nanoseconds / 1e9

    def distance_since_phase_start(self):
        """
        Distance traveled (via odometry) since the current avoid_phase began.
        Returns None if odometry isn't available, so callers can fall back to
        a time-based threshold instead.
        """
        if (self.current_x is None or self.current_y is None
                or self.phase_start_x is None or self.phase_start_y is None):
            return None

        return math.hypot(
            self.current_x - self.phase_start_x,
            self.current_y - self.phase_start_y
        )

    def start_avoidance(self, command):
        if not self.avoidance_enabled:
            self.state = 'STOPPED'
            self.get_logger().info(f'Avoidance disabled -- stopping from command {command}')
            return

        self.state = 'AVOIDING'
        self.avoid_phase = 'BRAKE'
        self.avoid_phase_start = self.get_clock().now()
        self.phase_start_x = self.current_x
        self.phase_start_y = self.current_y

        # Important fix:
        # Previously only an explicit STOP command triggered a reverse before
        # turning. But pivoting in place near a close obstacle can swing the
        # chassis corners into it even when the front-center depth reading
        # looked survivable -- the corners sweep through more lateral space
        # than the center point does. Since YOLO can't reliably see a person
        # up close anyway, depth is the real safety net here: ANY avoidance
        # that starts while something is inside hard_stop_distance backs up
        # first, regardless of which command triggered it.
        close_range = (
            self.have_depth
            and self.depth_is_fresh()
            and self.front_min_depth < self.hard_stop_distance
        )
        self.emergency_reverse = (command == 'STOP') or close_range

        # Important fix:
        # STOP used to always turn left.
        # Now STOP uses depth to choose whichever side is less blocked.
        if command == 'STOP' and self.have_depth and self.depth_is_fresh():
            chosen = self.choose_depth_avoid_direction()
        else:
            chosen = command

        if chosen == 'AVOID_RIGHT':
            self.avoid_turn_direction = -1.0
        else:
            self.avoid_turn_direction = 1.0

        self.get_logger().info(
            f'Entering AVOIDING from {command}: '
            f'chosen_turn={chosen}, phase=BRAKE, '
            f'turn_direction={self.avoid_turn_direction}, '
            f'emergency_reverse={self.emergency_reverse}, '
            f'front={self.front_min_depth:.2f}m, '
            f'left={self.left_min_depth:.2f}m, right={self.right_min_depth:.2f}m'
        )

    def compute_goal_seeking_twist(self):
        twist = Twist()

        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            if self.drive_without_odom:
                if not self.has_warned_no_odom:
                    self.get_logger().warn(
                        'No odometry received yet. Driving straight because drive_without_odom=True.'
                    )
                    self.has_warned_no_odom = True

                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0
                return twist

            if not self.has_warned_no_odom:
                self.get_logger().warn(
                    'No odometry received yet. Staying still because drive_without_odom=False.'
                )
                self.has_warned_no_odom = True

            return twist

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance_to_goal = math.hypot(dx, dy)

        if distance_to_goal < self.goal_tolerance:
            self.state = 'ARRIVED'
            self.get_logger().info(
                f'Goal reached: distance={distance_to_goal:.2f}m < tolerance={self.goal_tolerance:.2f}m'
            )
            return twist

        desired_yaw = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_yaw - self.current_yaw)

        angular = self.heading_kp * heading_error
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))

        speed_scale = max(0.5, math.cos(heading_error))
        linear = self.forward_speed * speed_scale

        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    # ---------------- Main control loop ----------------

    def publish_control(self):
        enable_msg = Bool()
        enable_msg.data = True
        self.enable_publisher.publish(enable_msg)

        twist = Twist()

        if self.state == 'DRIVING':
            depth_safety_command = self.get_depth_safety_command()

            if depth_safety_command in ('STOP', 'AVOID_LEFT', 'AVOID_RIGHT'):
                self.get_logger().info(
                    f'Depth safety override: {depth_safety_command}. '
                    f'YOLO/planner said {self.latest_command}. '
                    f'front={self.front_min_depth:.2f}m, '
                    f'left={self.left_min_depth:.2f}m, right={self.right_min_depth:.2f}m'
                )

                self.start_avoidance(depth_safety_command)
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            elif self.latest_command in ('CRUISE', 'GO'):
                twist = self.compute_goal_seeking_twist()

            elif self.latest_command in ('AVOID_LEFT', 'AVOID_RIGHT', 'STOP'):
                self.start_avoidance(self.latest_command)
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        elif self.state == 'AVOIDING':
            elapsed = self.seconds_since(self.avoid_phase_start)

            if self.avoid_phase == 'BRAKE':
                twist.linear.x = 0.0
                twist.angular.z = 0.0

                if elapsed >= self.brake_duration:
                    if self.emergency_reverse:
                        self.avoid_phase = 'REVERSE'
                    else:
                        self.avoid_phase = 'TURN'
                        self.turn_entry_front_depth = self.front_min_depth
                        self.turn_direction_checked = False

                    self.avoid_phase_start = self.get_clock().now()
                    self.phase_start_x = self.current_x
                    self.phase_start_y = self.current_y
                    self.get_logger().info(f'Avoidance phase -> {self.avoid_phase}')

            elif self.avoid_phase == 'REVERSE':
                # Important fix: gated on distance traveled, not a fixed
                # ~0.7s (~11cm) duration. Falls back to a longer fixed time
                # only if odometry isn't available.
                traveled = self.distance_since_phase_start()
                distance_met = traveled is not None and traveled >= self.reverse_min_distance
                no_odom_done = traveled is None and elapsed >= 1.8
                time_expired = elapsed >= self.reverse_max_duration

                if distance_met or no_odom_done or time_expired:
                    self.avoid_phase = 'TURN'
                    self.avoid_phase_start = self.get_clock().now()
                    self.phase_start_x = self.current_x
                    self.phase_start_y = self.current_y
                    self.turn_entry_front_depth = self.front_min_depth
                    self.turn_direction_checked = False
                    self.get_logger().info(
                        f'Avoidance phase -> TURN '
                        f'(reversed {traveled:.2f}m)' if traveled is not None
                        else 'Avoidance phase -> TURN (no odom, time-based reverse)'
                    )
                else:
                    twist.linear.x = self.reverse_speed
                    twist.angular.z = 0.0

            elif self.avoid_phase == 'TURN':
                # Important fix ("the correcting goes the opposite
                # direction"): choose_depth_avoid_direction picks the correct
                # side on paper, but if this vehicle's cmd_vel angular.z sign
                # convention (or the depth image's left/right) doesn't match
                # what the math assumes, the robot ends up steering further
                # INTO the obstacle instead of away from it. Rather than
                # guess at the sign convention, check partway through the
                # turn whether front distance is actually improving. If it
                # got worse, flip direction once and keep going.
                turn_check_time = 0.5
                if (not self.turn_direction_checked
                        and elapsed >= turn_check_time
                        and self.have_depth and self.depth_is_fresh()):
                    self.turn_direction_checked = True
                    if self.front_min_depth < self.turn_entry_front_depth - 0.05:
                        self.avoid_turn_direction *= -1.0
                        self.get_logger().warn(
                            f'TURN made things worse (front went '
                            f'{self.turn_entry_front_depth:.2f}m -> '
                            f'{self.front_min_depth:.2f}m). Flipping turn direction.'
                        )

                # Turn in place.
                twist.linear.x = 0.0
                twist.angular.z = self.avoid_turn_direction * self.turn_speed

                # Important fix:
                # Keep turning until the footprint corridor is clear enough,
                # but don't turn forever. Require several consecutive clear
                # ticks (not just one instantaneous reading) before trusting
                # it -- a single noisy good frame isn't proof the path is
                # actually clear, and ending the turn too early is exactly
                # what leads to a straight-line creep clipping the obstacle.
                if elapsed >= self.min_turn_duration:
                    if self.depth_corridor_clear_enough():
                        self.turn_clear_streak += 1
                    else:
                        self.turn_clear_streak = 0

                    clear_enough = self.turn_clear_streak >= 3

                    if clear_enough:
                        self.avoid_phase = 'CREEP'
                        self.avoid_phase_start = self.get_clock().now()
                        self.phase_start_x = self.current_x
                        self.phase_start_y = self.current_y
                        self.turn_clear_streak = 0
                        self.get_logger().info(
                            f'Avoidance phase -> CREEP, corridor clear: front={self.front_min_depth:.2f}m'
                        )

                    elif elapsed >= self.max_turn_duration:
                        self.avoid_phase = 'CREEP'
                        self.avoid_phase_start = self.get_clock().now()
                        self.phase_start_x = self.current_x
                        self.phase_start_y = self.current_y
                        self.turn_clear_streak = 0
                        self.get_logger().info(
                            f'Avoidance phase -> CREEP due to max turn time, '
                            f'front still={self.front_min_depth:.2f}m'
                        )

            elif self.avoid_phase == 'CREEP':
                # Important fix (this was the main "gets stuck" bug):
                # CREEP used to end after a fixed ~0.8s (~5-6cm of travel),
                # then immediately hand off to goal-seeking -- which would
                # swing the heading back toward the goal, re-enter the same
                # depth safety corridor, and repeat BRAKE->TURN->CREEP in
                # place forever. Now CREEP runs until the robot has actually
                # traveled creep_clearance_distance (via odometry), with
                # creep_max_duration as a hard cap. If that cap is hit and the
                # robot barely moved, it's very likely physically stuck
                # (wedged against the obstacle) -- stop cleanly and say so
                # instead of silently pretending it cleared.
                depth_safety_command = self.get_depth_safety_command()

                if depth_safety_command in ('STOP', 'AVOID_LEFT', 'AVOID_RIGHT'):
                    self.get_logger().info(
                        f'CREEP blocked by depth: {depth_safety_command}. '
                        f'front={self.front_min_depth:.2f}m, '
                        f'left={self.left_min_depth:.2f}m, right={self.right_min_depth:.2f}m'
                    )
                    self.start_avoidance(depth_safety_command)
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                else:
                    traveled = self.distance_since_phase_start()
                    distance_met = (
                        traveled is not None and traveled >= self.creep_clearance_distance
                    )
                    no_odom_fallback_done = (
                        traveled is None and elapsed >= self.creep_fallback_duration
                    )
                    time_expired = elapsed >= self.creep_max_duration

                    if distance_met or no_odom_fallback_done or time_expired:
                        if time_expired and not distance_met and traveled is not None and traveled < 0.10:
                            self.get_logger().warn(
                                f'CREEP made almost no progress ({traveled:.2f}m) after '
                                f'{self.creep_max_duration:.1f}s -- likely physically stuck '
                                f'against the obstacle. Stopping instead of resuming.'
                            )
                            self.state = 'STOPPED'
                            self.avoid_phase = None
                            self.avoid_phase_start = None
                            self.emergency_reverse = False
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            self.velocity_publisher.publish(twist)
                            return

                        self.state = 'DRIVING'
                        self.avoid_phase = None
                        self.avoid_phase_start = None
                        self.emergency_reverse = False

                        # Clear stale obstacle command after avoidance.
                        self.latest_command = 'CRUISE'

                        self.get_logger().info(
                            'Avoidance complete -> DRIVING '
                            f'(creep traveled {traveled:.2f}m)' if traveled is not None
                            else 'Avoidance complete -> DRIVING (no odom, time-based creep)'
                        )
                        twist = self.compute_goal_seeking_twist()
                    else:
                        twist.linear.x = self.creep_speed
                        # Important fix (the "scrapes the person" bug):
                        # This used to go perfectly straight (angular.z=0)
                        # once TURN finished. If TURN only just barely
                        # cleared the front-center reading, a dead-straight
                        # creep can still clip the obstacle on the side --
                        # the forward depth camera has no visibility once the
                        # obstacle is beside the robot, so nothing here could
                        # have caught that. Keep gently arcing away from the
                        # obstacle direction the whole time so lateral
                        # clearance keeps growing throughout the pass instead
                        # of being locked in by whatever the turn achieved.
                        twist.angular.z = self.avoid_turn_direction * self.creep_turn_bias

            else:
                self.get_logger().warn('Avoiding state had invalid phase. Resetting to DRIVING.')
                self.state = 'DRIVING'
                self.avoid_phase = None
                self.avoid_phase_start = None
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        elif self.state == 'STOPPED':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        elif self.state == 'ARRIVED':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        else:
            self.get_logger().warn(f'Unknown state {self.state}. Stopping.')
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.velocity_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RobotDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()