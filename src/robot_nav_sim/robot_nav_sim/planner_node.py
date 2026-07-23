import rclpy
from rclpy.node import Node
from robot_nav_interfaces.msg import Detection
from std_msgs.msg import String


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self.subscription = self.create_subscription(
            Detection,
            'detections',
            self.listener_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            String,
            'cmd',
            10
        )

        self.state = "CRUISE"

        # Tune these later after testing in Gazebo
        self.trigger_distance_m = 2.0
        self.min_confidence = 0.35

        # FIXED: was 640, but the RGBD camera in the world file is
        # configured at 320x240 (see reactive_avoidance_demo.sdf,
        # sensor name="rgbd_camera" -> <image><width>320</width>).
        # With the old 640 value, every bbox center between 160-320px was
        # being misclassified as "left of center" when it was actually
        # right-of-center in the real 320px-wide frame, silently flipping
        # AVOID_LEFT/AVOID_RIGHT for roughly half of all detections.
        # NOTE: this is still a hardcoded constant, not derived from the
        # actual image -- if the camera resolution in the SDF ever changes,
        # this must be updated to match, or better, the Detection message
        # should carry image width/height so this isn't a silent coupling
        # between two files that have to be kept in sync by hand.
        self.image_width = 320

        # Watchdog: if we haven't heard from perception at all in this long,
        # something upstream has actually died (not just "no obstacle seen"
        # -- publish_no_detection() in perception_node covers that case).
        # Falling back to CRUISE here is a deliberate safety choice: on a
        # real robot, "stop and freeze forever" is arguably safer than
        # "keep cruising blind," but for this sim/demo we favor not getting
        # permanently stuck. Revisit this tradeoff before real hardware.
        self.detection_timeout_s = 1.0
        self.last_detection_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.2, self.watchdog_callback)

    def watchdog_callback(self):
        elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if elapsed > self.detection_timeout_s:
            self.get_logger().warn(
                f'No detections (not even heartbeats) for {elapsed:.1f}s -- '
                f'perception may be down. Falling back to CRUISE.'
            )
            command_msg = String()
            command_msg.data = "CRUISE"
            self.state = "CRUISE"
            self.publisher_.publish(command_msg)
            # Reset so we only warn once per timeout window, not every 0.2s.
            self.last_detection_time = self.get_clock().now()

    def listener_callback(self, detection):
        self.last_detection_time = self.get_clock().now()

        command_msg = String()

        bbox_center_x = (detection.x_min + detection.x_max) / 2.0
        image_center_x = self.image_width / 2.0

        # Perception now publishes distance_m = -1.0 as an explicit sentinel
        # when it couldn't get a valid depth reading (out of range, hole in
        # the depth map, etc) instead of silently guessing. We treat that as
        # "not enough information to trigger avoidance" rather than either
        # assuming it's safe or assuming it's dangerous -- log it so it's
        # visible during tuning, since a real system probably shouldn't stay
        # silent about missing depth near obstacles.
        has_valid_distance = detection.distance_m > 0.0

        obstacle_is_relevant = (
            detection.confidence >= self.min_confidence
            and has_valid_distance
            and detection.distance_m < self.trigger_distance_m
        )

        if not has_valid_distance and detection.confidence >= self.min_confidence:
            self.get_logger().warn(
                f'Detected {detection.class_name} with confidence '
                f'{detection.confidence:.2f} but no valid depth reading '
                f'(distance_m={detection.distance_m}); not triggering avoidance '
                f'on this frame.'
            )

        if obstacle_is_relevant:
            self.state = "AVOID"

            if bbox_center_x < image_center_x:
                command_msg.data = "AVOID_RIGHT"
            else:
                command_msg.data = "AVOID_LEFT"
        else:
            self.state = "CRUISE"
            command_msg.data = "CRUISE"

        self.publisher_.publish(command_msg)

        self.get_logger().info(
            f'state={self.state}, class={detection.class_name}, '
            f'conf={detection.confidence:.2f}, dist={detection.distance_m:.2f}, '
            f'bbox_center_x={bbox_center_x:.1f} → Publishing {command_msg.data}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = PlannerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()