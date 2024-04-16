import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math
class TeleopSerialNode(Node):
    def __init__(self):
        super().__init__('teleop_serial_node')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.subscription 
        self.serial_port = serial.Serial(port = '/dev/ttyAMA1', baudrate = 115200, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS)

    def listener_callback(self, msg):
        speed = msg.linear.x
        rotation = msg.angular.z
        serial_data = f"{speed:.2f}\n"
        self.serial_port.write(serial_data.encode())
        self.get_logger().info(f"Sending to STM32: {serial_data.strip()}")
        

def main(args=None):
    rclpy.init(args=args)
    teleop_serial_node = TeleopSerialNode()
    rclpy.spin(teleop_serial_node)
    # Cleanup
    teleop_serial_node.serial_port.close()
    teleop_serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
