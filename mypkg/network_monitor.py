import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class NetworkMonitor(Node):
    def __init__(self):
        super().__init__('network_monitor')

        self.declare_parameter('check_interval', 5.0)
        interval = self.get_parameter('check_interval').value

        self.publisher = self.create_publisher(
            String, 'network_status', 10)
        self.timer = self.create_timer(interval, self.check_network)

    def check_network(self):
        result = subprocess.run(
            ['ping', '-c', '1', '8.8.8.8'],
            stdout=subprocess.DEVNULL)

        msg = String()
        msg.data = 'ONLINE' if result.returncode == 0 else 'OFFLINE'
        self.publisher.publish(msg)

        self.get_logger().info(f'Network: {msg.data}')

def main():
    rclpy.init()
    node = NetworkMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
