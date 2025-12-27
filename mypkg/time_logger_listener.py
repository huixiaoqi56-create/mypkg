import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import csv
import os
import time
from datetime import datetime


class TimeLogger(Node):
    def __init__(self):
        super().__init__('time_logger')

        # パラメータ追加
        self.declare_parameter('log_path', 'time_log.csv')
        self.declare_parameter('offline_timeout', 3.0)

        self.log_path = self.get_parameter('log_path').value
        self.offline_timeout = self.get_parameter('offline_timeout').value

        # ログ保存先ディレクトリ作成
        log_dir = os.path.dirname(self.log_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)

        # Subscriber
        self.create_subscription(
            Int32,
            'elapsed_time',
            self.callback,
            10
        )

        # CSV 初期化
        if not os.path.exists(self.log_path):
            with open(self.log_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'elapsed_seconds'])

        # 最終受信時刻
        self.last_received_time = time.time()

        self.watchdog_timer = self.create_timer(
            1.0, self.watchdog_callback)

        # オフライン監視タイマー
        self.watchdog_timer = self.create_timer(
            1.0, self.watchdog_callback)

        self.get_logger().info(
            f'Logging to {self.log_path}')
        self.get_logger().info(
            f'Offline timeout = {self.offline_timeout} sec')

    def callback(self, msg):
        now = datetime.now().isoformat()

        with open(self.log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([now, msg.data])

        self.last_received_time = time.time()

        self.get_logger().info(
            f'Logged: {msg.data} sec')

    def watchdog_callback(self):
        elapsed = time.time() - self.last_received_time

        self.get_logger().info(
            f'[WATCHDOG] elapsed = {elapsed:.2f} sec')

        if elapsed > self.offline_timeout:
           self.get_logger().error(
            'ALARM: battery_simulator is OFFLINE!')
           print('\a')
           rclpy.shutdown()


def main():
    rclpy.init()
    node = TimeLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

