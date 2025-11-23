#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose
import math
from std_msgs.msg import String




class FollowersNode(Node):
    def __init__(self):
        super().__init__("followers_node")

        self.followers_names =[]
        self.followers_pose = {}
        self.followers_publishers = {}


        self.create_subscription(String, '/new_follower' , self.append_follower , 10)
        self.create_subscription(Pose, '/turtle1/pose' , self.leader_callback , 10)



    def append_follower(self, msg):
        name = msg.data
        self.followers_names.append(name)
        self.followers_pose[name] = None

        callback = self.make_pose_callback(name)
        self.create_subscription(Pose, f"/{name}/pose", callback, 10)

        self.get_logger().info(f"subscribtion created for {name} ")
        self.followers_publishers[name] = self.create_publisher(Twist , f"/{name}/cmd_vel" , 10)
        self.get_logger().info(f"publisher created for {name} ")

    

    def make_pose_callback(self, name):
        def callback(msg):
            self.followers_pose[name] = msg
            self.get_logger().info(f"{name} pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}")
        return callback


    

    def leader_callback(self, pose):
        leader_pose = pose


        for name in self.followers_names:
            cmd = Twist()
            f_pose  = self.followers_pose[name]
            if f_pose is None: # if the follower just spawned and pos still not arrived
                continue

            f_pub = self.followers_publishers[name]

            dx = leader_pose.x - f_pose.x 
            dy = leader_pose.y - f_pose.y 

            distance = math.sqrt(dx**2 + dy**2)

            angle_to_leader = math.atan2(dy, dx) # absolute angle between leader and dollower
            angle_error = angle_to_leader - f_pose.theta  # difference between where the follower is pointing and where it should face
            angle_error = math.atan2(math.sin(angle_error) , math.cos(angle_error)) # wrapping angle_error to [-π, π] ensures shortest rotation

            safe_distance = 1.0 # keep distance from the leader
            max_speed = 2.5 

            cmd.linear.x = min(max_speed, max(0.0 , (distance - safe_distance)))

            cmd.angular.z = 2.0 * angle_error # scale the turning speed 

            self.get_logger().info(f"Publishing to {name}: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}, distance={distance:.2f}, angle_error={angle_error:.2f}")
            f_pub.publish(cmd)


        




def main(args=None):
    rclpy.init(args=args)
    node =  FollowersNode()
    rclpy.spin(node)
    rclpy.shutdown()