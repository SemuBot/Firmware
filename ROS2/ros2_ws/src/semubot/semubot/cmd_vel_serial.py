import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import serial.tools.list_ports

class CmdVelSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_serial')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.send_to_serial, 10)
        
        # Find the available serial port (either ACM0 or ACM1)
        self.serial_port = self.find_serial_port()

        if self.serial_port is None:
            self.get_logger().error('No valid serial port found. Exiting.')
            rclpy.shutdown()
            return

        self.serial_port = serial.Serial(self.serial_port, 115200, timeout=1)
        self.get_logger().info(f'Using serial port: {self.serial_port.name}')

    def find_serial_port(self):
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if port.device in ['/dev/ttyACM0', '/dev/ttyACM1']:
                return port.device
        return None

    def send_to_serial(self, msg):
        data = f"{{linear_x:{msg.linear.x:.2f},angular_z:{msg.angular.z:.2f}}}\n"
        self.serial_port.write(data.encode())
        self.get_logger().info(f'Sent to STM32: {data.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerial()
    if node.serial_port is not None:
        rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

