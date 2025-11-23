#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk
from turtlesim.srv import Spawn
import random
from std_msgs.msg import String
from turtlesim.msg import Pose 
from math import cos, sin


class LeaderGUI(Node):
    def __init__(self):
        super().__init__("leader_gui")
        self.follower_counter = 1
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose = None
        self.pose_sub = self.create_subscription(Pose,'/turtle1/pose', self.pose_callback, 10)
        self.cli = self.create_client(Spawn, '/spawn')
        self.spawn_pub = self.create_publisher(String, "/new_follower", 10)

       
        # Tkinter GUI
        self.root = tk.Tk()
        self.root.title("Leader turtle GUI")
        self.root.geometry("500x300")  

        frame = tk.Frame(self.root)
        frame.pack()
 
        tk.Label(frame, text="Linear X").grid(row=1, column=0) 
        self.linear_slider = tk.Scale(frame, from_=-5, to=5, resolution=0.1, orient=tk.HORIZONTAL)
        self.linear_slider.grid(row=1, column=1) 
        self.linear_label = tk.Label(frame, text=f"{self.linear_slider.get():.2f}")
        self.linear_label.grid(row=1, column=2) 

        tk.Label(frame, text="Angular Z").grid(row=2, column=0) 
        self.angular_slider = tk.Scale(frame, from_=-3, to=3, resolution=0.1, orient=tk.HORIZONTAL)
        self.angular_slider.grid(row=2, column=1) 
        self.angular_label = tk.Label(frame, text=f"{self.angular_slider.get():.2f}")
        self.angular_label.grid(row=2, column=2)



        tk.Button(frame, text="Spawn Follower", command=self.spawn_turtle).grid(row=3, column=0, columnspan=3, pady=10)



        # Handle closing the window
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Start periodic updates
        self.root.after(100, self.update)
    
    def pose_callback(self, msg):
        self.pose = msg


    def update(self):
        self.send_velocity()
        rclpy.spin_once(self, timeout_sec=0)  
        if rclpy.ok():
            self.root.after(100, self.update)
        else:
            self.on_close()


    def on_close(self):
        self.get_logger().info("Shutting down GUI")
        self.root.destroy()
        rclpy.shutdown()

    def spawn_turtle(self):
        if not self.cli.service_is_ready():
            self.get_logger().warn("Spawn service not ready")
            return
        
        x = random.uniform(2, 8)
        y = random.uniform(2, 8)
        theta = random.uniform(0, 6.28)
        self.follower_counter += 1
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = f"follower{self.follower_counter}"
        future = self.cli.call_async(request)
        future.add_done_callback(self.spawn_response)

    def spawn_response(self, future):
        try:
            response = future.result()
            self.spawn_pub.publish(String(data=response.name))
            self.get_logger().info(f"Spawned new turtle {response.name}")
        except Exception as e:
            self.get_logger().error(f"Spawn service failed: {e}")

    def send_velocity(self):
        if self.pose is None:
            return

        x,y = self.pose.x , self.pose.y
        linear = self.linear_slider.get()
        angular = self.angular_slider.get()

        # Stop forward movement if heading toward right or top wall
        if (x <= 1.0 and cos(self.pose.theta) < 0) or (x >= 10.0 and cos(self.pose.theta) > 0):
            linear = 0.0
        if (y <= 1.0 and sin(self.pose.theta) < 0) or (y >= 10.0 and sin(self.pose.theta) > 0):
            linear = 0.0


        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)
        self.get_logger().info(f"Published: linear = {msg.linear.x}, angular = {msg.angular.z}")


def main(args=None):
    rclpy.init(args=args)
    node = LeaderGUI()
    try:
        node.root.mainloop() 
    finally:
        rclpy.shutdown()
