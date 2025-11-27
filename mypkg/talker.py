import rclpy
from rclpy.node import Node
from person_msgs.srv import Query                                       #from std_msgs.msg import Person

rclpy.init()
node = Node( "talker")
#pub = node.create_publisher(Int16, "person", 10)
#n=0
def cb(request,response):
    #global n
    #msg = Person()
    #msg.name = "hakozaki":
    if request.name == "hakozaki":
       response.age == 46
    else:
        response.age =255
    return response

#    msg.age = n
 #   pub.publish(msg)
  #  n += 1

def main():
   # node.create_timer(0.5, cb)
    srv = node.create_service(Query, "query",cb)
    rclpy.spin(node)
