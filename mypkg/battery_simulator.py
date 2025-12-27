import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')
        self.publisher = self.create_publisher(Int32, 'elapsed_time', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.elapsed = 0

    def timer_callback(self):
        msg = Int32()
        msg.data = self.elapsed
        self.publisher.publish(msg)
        self.get_logger().info(f'Elapsed time: {self.elapsed} sec')
        self.elapsed += 1


def main():
    rclpy.init()
    node = BatterySimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

