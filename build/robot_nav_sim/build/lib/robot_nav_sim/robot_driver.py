import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        self.subscription = self.create_subscription(
            String,
            'cmd',      # must match perception_node's topic name exactly
            self.listener_callback,
            10
        )


    def listener_callback(self, msg):
        if msg.data == "STOP":
            print("Stopping")
        else:
            print("Driving")

def main(args=None):
    rclpy.init(args=args)
    node = RobotDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()