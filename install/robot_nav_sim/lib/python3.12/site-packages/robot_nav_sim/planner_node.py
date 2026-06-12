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
        self.publisher_ = self.create_publisher(String, 'cmd', 10)  # what topic does planner publish to?

    def listener_callback(self, detection):
        command_msg = String()

        if detection.confidence > 0.5 and detection.distance_m < 2.0:
            command_msg.data = "STOP"
        else:
            command_msg.data = "GO"

        self.publisher_.publish(command_msg)
        self.get_logger().info(
            f'Received {detection.class_name} at {detection.distance_m}m → Publishing {command_msg.data}'
        )
        msg = String()
        msg.data = command_msg              
        self.publisher_.publish(msg)                            
        self.get_logger().info(f'Publishing command: {command_msg}')

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()