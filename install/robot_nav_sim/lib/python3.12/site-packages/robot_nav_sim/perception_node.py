import rclpy
from rclpy.node import Node
from robot_nav_interfaces.msg import Detection

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')         
        self.publisher_ = self.create_publisher(Detection, 'detections', 10)         
        self.timer = self.create_timer(0.5, self.timer_callback)              

    def timer_callback(self):
        msg = Detection()
        msg.class_name = "person"
        msg.x_min = 120
        msg.y_min = 80
        msg.x_max = 260
        msg.y_max = 300
        msg.confidence = 0.91
        msg.distance_m = 1.7

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing detection: {msg.class_name}, conf={msg.confidence}, dist={msg.distance_m}m'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()