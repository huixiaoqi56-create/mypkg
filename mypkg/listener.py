import rclpy
from rclpy.node import Node
from std_msgs.srv import Query

rclpy.init()
node = Node( "listener")
#def cb(msg):
 #   global node
  #  node.get_logger().info("Listen: %d" % msg.data)

def main():
   # pub = node.create_subscription(Person, "person", cb, 10)
    #rclpy.spin(node)
   client = node.create_client(Query,'query')
   while not client.wait_for_service(timeout_sec=1.0):
       node.get_logger().info('待機中')
   req = Query.Request()
   req.name = "hakozaki"
   future = client .call_async(req)
   while rclpy.ok():
       rclpy.spin_once(node)
       if future.done():
          try:
              responce = future.result()
          except:
              node.get_logger().info('呼び出し失敗')
          else:
              node.get_logger().info("agt: {}" .format(response.age))
          break
   node.destroy_node()
   rclpy.shutdown()
