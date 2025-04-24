#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
from rclpy.qos import QoSProfile


class SerialCmdVel(Node):
    def __init__(self):
        """
        Sub to /cmd_vel & sends vel cmds over a serial connection (to an Arduino that controls motors).
        """
        super().__init__('serial_cmd_vel')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        try:
            # Open a serial connection to the device
            self.serial = serial.Serial(port, baudrate=baud, timeout=1.0)
            self.get_logger().info(f"Connected to {port} at {baud} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            rclpy.shutdown()
            return

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile=QoSProfile(depth=1)
        )
        # Used to limit the send rate to avoid flooding the serial connection
        self.last_send_time = time.time()

    def cmd_vel_callback(self, msg: Twist):
        now = time.time()
        # Skips sending if less than 30ms passed since the last msg (limits to ~33Hz max)
        if now - self.last_send_time < 0.03:  
            return  

        self.last_send_time = now

        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        command_str = f"{vx:.3f} {vy:.3f} {wz:.3f}\n"
        try:
            self.serial.write(command_str.encode()) # # Sends cmd via serial
            self.serial.flush()  # ensures buffer is written out immediately
            self.get_logger().info(f"Sent: {command_str.strip()}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
