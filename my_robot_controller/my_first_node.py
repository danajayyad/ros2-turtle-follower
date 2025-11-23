#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node


# Node is the base class for all ROS 2 nodes
# MyNode that inherits from Node.
# subclass of Node
class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.counter = 0
        self.create_timer(1.0 , self.timer_callback)


    def timer_callback(self):
        self.get_logger().info("hello" + str(self.counter))
        self.counter+=1

def main(args=None):
    rclpy.init(args=args) # starts the ros2 middleware communication layer before creating nodes
    node = MyNode() # create node

    rclpy.spin(node)  # spin make the nodes stay alive until killing it, to keep the communication with other nodes

    rclpy.shutdown()  # destroy the node 
    



if __name__ =='__main__':
    main()