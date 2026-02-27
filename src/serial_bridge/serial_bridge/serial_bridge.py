#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import sys

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.get_logger().info(f'Connecting to Arduino on {port} at {baud} baud')

        try:
            self.serial_conn = serial.Serial(port, baud, timeout=1, write_timeout=1)
            time.sleep(2) # Wait for connection to stabilize
            self.get_logger().info('Connected to Arduino')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            sys.exit(1)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, 'raw_vel', 10)

        # Timer to read from serial
        self.timer = self.create_timer(0.01, self.read_serial)

    def cmd_vel_callback(self, msg):
        command = f'v {msg.linear.x:.2f} {msg.linear.y:.2f} {msg.angular.z:.2f}\n'
        #print(command)
        try:
            self.serial_conn.write(command.encode('utf-8'))
            # self.get_logger().info(f'Sent: {command.strip()}')
        except serial.SerialTimeoutException:
            self.get_logger().warn('Serial write timed out')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def read_serial(self):
        if self.serial_conn.in_waiting > 0:
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line:
                    if line.startswith('v'):
                         parts = line.split()
                         if len(parts) == 4:
                             try:
                                 msg = Twist()
                                 msg.linear.x = float(parts[1])
                                 msg.linear.y = float(parts[2])
                                 msg.angular.z = float(parts[3])
                                 self.publisher_.publish(msg)
                             except ValueError:
                                 self.get_logger().warn(f'Failed to parse velocity data: {line}')
                    elif line.startswith('i'):
                         pass
                    else:
                        pass
                        #self.get_logger().info(f'Arduino: {line}')
            except Exception as e:
                self.get_logger().warning(f'Serial read error: {e}')

def main(args=None):
    rclpy.init(args=args)

    bridge = SerialBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.serial_conn.close()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
