import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
import time
import os
import sys

class SystemHealthListener(Node):
    def __init__(self):
        super().__init__('system_health_listener')

        self.declare_parameter('offline_timeout', 3.0)
        self.declare_parameter('log_path', '/tmp/system_health_log.csv')

        self.timeout = self.get_parameter('offline_timeout').value
        self.log_path = self.get_parameter('log_path').value

        if os.path.dirname(self.log_path):
            os.makedirs(os.path.dirname(self.log_path), exist_ok=True)

        with open(self.log_path, 'w') as f:
            f.write('time_sec,network_ok\n')

        self.last_time = time.time()
        self.sec = None
        self.net = None

        self.create_subscription(Int32, 'system_seconds', self.sec_cb, 10)
        self.create_subscription(Bool, 'network_status', self.net_cb, 10)

        self.timer = self.create_timer(1.0, self.check_offline)
        self.get_logger().info('SystemHealthListener started')

    def sec_cb(self, msg):
        self.sec = msg.data
        self.last_time = time.time()
        self.try_log()
        self.show()

    def net_cb(self, msg):
        self.net = msg.data
        self.last_time = time.time()
        self.try_log()
        self.show()

    def try_log(self):
        if self.sec is None or self.net is None:
            return
        with open(self.log_path, 'a') as f:
            f.write(f'{self.sec},{int(self.net)}\n')

    def show(self):
        if self.sec is None or self.net is None:
            return
        self.get_logger().info(
            f'time={self.sec}s network={"OK" if self.net else "NG"}'
        )

    def check_offline(self):
        if time.time() - self.last_time > self.timeout:
            self.get_logger().error('OFFLINE detected. Exit.')
            sys.exit(1)

def main():
    rclpy.init()
    node = SystemHealthListener()
    rclpy.spin(node)
