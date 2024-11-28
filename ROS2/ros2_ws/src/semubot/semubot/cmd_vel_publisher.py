import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(5, self.publish_cmd_vel)

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = random.uniform(-1.0, 1.0)
        msg.linear.y = random.uniform(-1.0, 1.0)
        msg.linear.z = random.uniform(-1.0, 1.0)
        msg.angular.x = random.uniform(-0.5, 0.5)
        msg.angular.y = random.uniform(-0.5, 0.5)
        msg.angular.z = random.uniform(-1.0, 1.0)
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Publishing: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), "
            f"angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

