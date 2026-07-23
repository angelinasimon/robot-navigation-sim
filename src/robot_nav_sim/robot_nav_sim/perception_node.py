import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

import message_filters
from sensor_msgs.msg import Image
from robot_nav_interfaces.msg import Detection

from cv_bridge import CvBridge
from ultralytics import YOLO


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.publisher_ = self.create_publisher(
            Detection,
            'detections',
            10
        )

        # Color and depth are subscribed together via message_filters so we
        # can look up real depth at the detected bbox instead of guessing.
        # Both come from the same RGBD sensor, so their timestamps should be
        # very close together -- an approximate sync with a small slop
        # window is enough to pair them correctly.
        self.color_sub = message_filters.Subscriber(self, Image, '/rgbd_camera/image')
        self.depth_sub = message_filters.Subscriber(self, Image, '/rgbd_camera/depth_image')

        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.time_synchronizer.registerCallback(self.image_callback)

        self.bridge = CvBridge()

        # Pretrained YOLO model.
        # 's' is slightly slower than 'n' but gives better detections.
        self.model = YOLO('yolov8s.pt')

        # Debug frame saving -- was every callback (self.debug_save_frame=True
        # unconditionally), which adds disk I/O to every single frame and
        # will skew your detection-latency numbers. Now throttled to every
        # Nth frame instead.
        self.debug_save_frame = True
        self.debug_save_every_n_frames = 30
        self._frame_count = 0

        # Half-width of the pixel patch sampled around the bbox center when
        # reading depth. A single pixel can be noisy or hit a hole in the
        # depth map; a small median patch is more robust.
        self.depth_sample_radius = 3

        self.get_logger().info(
            'Perception node started. Listening on /rgbd_camera/image '
            'and /rgbd_camera/depth_image (time-synchronized).'
        )

    def publish_no_detection(self):
        """Publish an explicit 'nothing relevant seen this frame' heartbeat.

        Without this, the planner only hears from us when YOLO finds a box,
        so the moment an obstacle leaves the frame (or was never seen), we
        go silent -- and the planner then never publishes a fresh CRUISE,
        leaving robot_driver stuck on whatever command it received last
        (e.g. a stale AVOID_RIGHT) forever. Publishing every frame, even
        empty ones, keeps the whole pipeline live.
        """
        detection = Detection()
        detection.class_name = ''
        detection.x_min = 0
        detection.y_min = 0
        detection.x_max = 0
        detection.y_max = 0
        detection.confidence = 0.0
        detection.distance_m = -1.0
        self.publisher_.publish(detection)

    def sample_depth_at_bbox(self, depth_frame, x1, y1, x2, y2):
        """Return distance in meters at the bbox center, using a small
        median-filtered patch to reduce noise. Returns None if no valid
        (finite, positive) depth samples are found in the patch.
        """
        height, width = depth_frame.shape[:2]

        center_x = int((x1 + x2) / 2.0)
        center_y = int((y1 + y2) / 2.0)

        r = self.depth_sample_radius
        x_lo = max(0, center_x - r)
        x_hi = min(width, center_x + r + 1)
        y_lo = max(0, center_y - r)
        y_hi = min(height, center_y + r + 1)

        patch = depth_frame[y_lo:y_hi, x_lo:x_hi].astype(np.float32)

        # Depth images from gz's RGBD sensor are typically 32FC1 in meters,
        # but can come through as uint16 millimeters depending on encoding.
        # Normalize to meters if it looks like a millimeter encoding.
        if depth_frame.dtype == np.uint16:
            patch = patch / 1000.0

        valid = patch[np.isfinite(patch) & (patch > 0.0)]

        if valid.size == 0:
            return None

        return float(np.median(valid))

    def image_callback(self, color_msg, depth_msg):
        start_time = time.time()

        try:
            frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as exc:
            # A single malformed frame should not permanently kill perception.
            self.get_logger().warn(f'Image conversion failed, skipping frame: {exc}')
            return

        self._frame_count += 1
        if self.debug_save_frame and self._frame_count % self.debug_save_every_n_frames == 0:
            cv2.imwrite('/tmp/live_debug.jpg', frame)

        try:
            results = self.model(frame, conf=0.25, verbose=False)
        except Exception as exc:
            self.get_logger().warn(f'YOLO inference failed, skipping frame: {exc}')
            return

        if len(results) == 0:
            self.publish_no_detection()
            return

        boxes = results[0].boxes

        if len(boxes) == 0:
            self.publish_no_detection()
            return

        # Publish the highest-confidence detection.
        best_box = max(boxes, key=lambda b: float(b.conf[0]))

        class_id = int(best_box.cls[0])
        class_name = self.model.names[class_id]
        confidence = float(best_box.conf[0])

        x1, y1, x2, y2 = best_box.xyxy[0].tolist()

        distance_m = self.sample_depth_at_bbox(depth_frame, x1, y1, x2, y2)

        if distance_m is None:
            # No valid depth in the patch (out of range, NaN/inf hole, etc).
            # Publishing an obviously-invalid sentinel rather than a fake
            # "safe" or "unsafe" number, so the planner can decide how to
            # treat missing depth explicitly instead of silently trusting it.
            distance_m = -1.0
            self.get_logger().warn(
                f'No valid depth found for {class_name} bbox center; '
                f'publishing distance_m=-1.0'
            )

        detection = Detection()
        detection.class_name = class_name
        detection.x_min = int(x1)
        detection.y_min = int(y1)
        detection.x_max = int(x2)
        detection.y_max = int(y2)
        detection.confidence = confidence
        detection.distance_m = distance_m

        self.publisher_.publish(detection)

        latency_ms = (time.time() - start_time) * 1000

        self.get_logger().info(
            f'Published {class_name} '
            f'(conf={confidence:.2f}) '
            f'bbox=({int(x1)}, {int(y1)}, {int(x2)}, {int(y2)}) '
            f'distance_m={distance_m:.2f} '
            f'latency={latency_ms:.1f} ms'
        )


def main(args=None):
    rclpy.init(args=args)

    node = PerceptionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()