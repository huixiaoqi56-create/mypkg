#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')
        self.publisher_ = self.create_publisher(Int32, 'uptime', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        self.get_logger().info('BatterySimulator started')

    def timer_callback(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing uptime: {self.counter}')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
