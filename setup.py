from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spawn_and_follow.launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='danayyad2111@gmail.com',
    description='Leader-follower turtlesim controller with Tkinter GUI',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "test_node=my_robot_controller.my_first_node:main",
            "draw_circle=my_robot_controller.draw_circle:main" ,
            "pose_subscriber=my_robot_controller.pose_subscriber:main",
            "turtle_controller=my_robot_controller.turtle_controller:main",
            "leader_gui=my_robot_controller.leader_gui:main" ,
            "followers_node=my_robot_controller.followers_node:main"


        ],
    },
)
