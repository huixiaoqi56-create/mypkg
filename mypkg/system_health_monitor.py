#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Hakozaki Teruki
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
import time
import socket

class SystemHealthMonitor(Node):
    def __init__(self):
        super().__init__('system_health_monitor')

        self.sec_pub = self.create_publisher(Int32, 'system_seconds', 10)
        self.net_pub = self.create_publisher(Bool, 'network_status', 10)

        self.start_time = time.time()
        self.timer = self.create_timer(1.0, self.timer_cb)

        self.get_logger().info('SystemHealthMonitor started')

    def timer_cb(self):
        sec = int(time.time() - self.start_time)
        net_ok = self.check_network()

        self.sec_pub.publish(Int32(data=sec))
        self.net_pub.publish(Bool(data=net_ok))

        self.get_logger().info(
            f"time={sec}s network={'OK' if net_ok else 'NG'}"
        )

    def check_network(self):
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=1)
            return True
        except:
            return False

def main():
    rclpy.init()
    node = SystemHealthMonitor()
    rclpy.spin(node)
