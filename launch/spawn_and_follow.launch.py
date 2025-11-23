from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),

        # Leader GUI node
        Node(
            package='my_robot_controller',
            executable='leader_gui',
            name='leader_gui',
            output='screen'
        ),

        # Followers controller node
        Node(
            package='my_robot_controller',
            executable='followers_node',
            name='followers_node',
            output='screen'
        ),
    ])
