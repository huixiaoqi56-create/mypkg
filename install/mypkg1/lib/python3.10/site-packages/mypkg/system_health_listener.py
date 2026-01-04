#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 hakozaki teruki
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
import time
import csv
import os

class SystemHealthListener(Node):
    def __init__(self):
        super().__init__('system_health_listener')

        self.declare_parameter('log_path', '/tmp/system_health_log.csv')
        self.log_path = self.get_parameter('log_path').value

        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        self.log_file = open(self.log_path, 'w', newline='')
        self.writer = csv.writer(self.log_file)
        self.writer.writerow(['uptime_sec', 'network_ok', 'recv_time'])

        self.last_recv_time = time.time()
        self.latest_net = True

        self.create_subscription(Int32, 'uptime_sec', self.cb_uptime, 10)
        self.create_subscription(Bool, 'network_ok', self.cb_net, 10)

        self.timer = self.create_timer(1.0, self.check_status)

        self.get_logger().info(f'SystemHealthListener started, log={self.log_path}')

    def cb_uptime(self, msg):
        self.last_recv_time = time.time()
        self.writer.writerow([msg.data, self.latest_net, self.last_recv_time])
        self.log_file.flush()
        self.get_logger().info(f'uptime={msg.data}s net={self.latest_net}')

    def cb_net(self, msg):
        self.latest_net = msg.data
        if not msg.data:
            self.get_logger().error('Network OFF detected (listener)')
            self.shutdown()

    def check_status(self):
        if time.time() - self.last_recv_time > 3.0:
            self.get_logger().error('Monitor OFFLINE detected')
            self.shutdown()

    def shutdown(self):
        self.log_file.close()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = SystemHealthListener()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
