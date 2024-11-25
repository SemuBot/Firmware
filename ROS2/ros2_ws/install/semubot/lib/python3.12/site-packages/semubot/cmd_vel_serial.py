import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_serial')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.send_to_serial, 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Replace with your port

    def send_to_serial(self, msg):
        data = f"{{linear_x:{msg.linear.x:.2f},angular_z:{msg.angular.z:.2f}}}\n"
        self.serial_port.write(data.encode())
        self.get_logger().info(f'Sent to STM32: {data.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerial()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
