#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Hakozaki Teruki
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
import time
import subprocess

class SystemHealthMonitor(Node):
    def __init__(self):
        super().__init__('system_health_monitor')

        self.uptime_pub = self.create_publisher(Int32, 'uptime_sec', 10)
        self.net_pub = self.create_publisher(Bool, 'network_ok', 10)

        self.start_time = time.time()
        self.timer = self.create_timer(1.0, self.tick)

        self.get_logger().info('SystemHealthMonitor started')

    def check_network(self):
        try:
            subprocess.check_call(
                ['ping', '-c', '1', '8.8.8.8'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            return True
        except subprocess.CalledProcessError:
            return False

    def tick(self):
        elapsed = int(time.time() - self.start_time)

        net_ok = self.check_network()

        self.uptime_pub.publish(Int32(data=elapsed))
        self.net_pub.publish(Bool(data=net_ok))

        self.get_logger().info(f'time={elapsed}s network={"OK" if net_ok else "NG"}')

        if not net_ok:
            self.get_logger().error('Network disconnected. shutting down.')
            rclpy.shutdown()

def main():
    rclpy.init()
    node = SystemHealthMonitor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
