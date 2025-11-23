#!/usr/bin/env python3
import rclpy
from rclpy.node import Node  # rclpy/node/Node
from turtlesim.msg import Pose  # turtlesim/msg/Pose
from geometry_msgs.msg import Twist # geometry_msgs/msg/Twist
from turtlesim.srv import SetPen # turtlesim/srv/SetPen
from functools import partial

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.previous_x = 0.0
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel" , 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose , "/turtle1/pose" ,  self.pose_callback, 10)
        self.get_logger().info("Turtle Contoller has been started")

    def pose_callback(self, pose):
        cmd = Twist()
        # prevent turle from touching the edges by turning
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0

        self.cmd_vel_publisher_.publish(cmd)

        # call the service based on how frequently we recieve a pose 
        if pose.x  > 5.5 and self.previous_x <= 5.5: # moving from left to right
            self.previous_x = pose.x
            self.get_logger().info("set color to red")
            self.call_set_pen_service(225, 0 , 0 , 3, 0)
        elif pose.x <= 5.5 and self.previous_x > 5.5: # moving form right to left 
            self.previous_x = pose.x
            self.get_logger().info("Set color to green")
            self.call_set_pen_service(0, 225, 0, 3, 0)
            





    # service call method 
    def call_set_pen_service(self , r, g, b, width , off):
        client = self.create_client(SetPen , "/turtle1/set_pen" )
        while not client.wait_for_service(1.0): #check each 1 sec if the service is not found 
            self.get_logger().warn("Waiting for service....")

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        # future.add_done_callback(partials(self.callback_set_pen))
        future.add_done_callback(self.callback_set_pen)

    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,)) # %r prints the full exception object




def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()