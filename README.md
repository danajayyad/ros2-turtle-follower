<<<<<<< HEAD
## Overview

This project is a **leader-follower controller for turtles in ROS 2**, using the **turtlesim simulator**.  

It consists of two main components:

1. **LeaderGUI Node** – A Tkinter-based GUI that allows interactive control of a leader turtle. Users can adjust the linear and angular velocity of the leader using sliders and spawn new follower turtles dynamically.  
2. **FollowersNode** – A node that subscribes to the poses of both the leader and the follower turtles, computes velocity commands for the followers, and ensures they follow the leader while maintaining a safe distance.

The system demonstrates **real-time interaction** in ROS 2, combining GUI control, service calls, and message passing.

---

## Logic & Math

### Leader Control

- The **LeaderGUI node** publishes `geometry_msgs/Twist` messages to `/turtle1/cmd_vel` based on slider input.  
- Users can control:  
  - **Linear X**: forward/backward speed  
  - **Angular Z**: rotational speed  

### Follower Behavior

When a follower is spawned:

1. The **FollowersNode** subscribes to the follower’s pose (`/followerX/pose`) and the leader’s pose (`/turtle1/pose`).  
2. It calculates the **distance** and **angle error** between the follower and the leader.

#### Distance computation

The Euclidean distance between the follower and the leader is calculated as:

\[
\text{distance} = \sqrt{(x_{\text{leader}} - x_{\text{follower}})^2 + (y_{\text{leader}} - y_{\text{follower}})^2}
\]

#### Angle computation

The angle to the leader is:

\[
\theta_{\text{to\_leader}} = \text{atan2}(y_{\text{leader}} - y_{\text{follower}}, x_{\text{leader}} - x_{\text{follower}})
\]

The **angle error** between the follower’s current heading and the direction to the leader is:

\[
\theta_{\text{error}} = \theta_{\text{to\_leader}} - \theta_{\text{follower}}
\]

- The angle is wrapped to \([-π, π]\) to ensure the shortest rotation.  

#### Velocity commands

- **Linear velocity:** Proportional to `(distance - safe_distance)`, capped at a maximum speed.  
- **Angular velocity:** Proportional to `angle_error` to steer the follower toward the leader smoothly.

This simple proportional control ensures followers maintain a **safe distance** while accurately following the leader’s path.

---

## Nodes

### 1. leader_gui

- Provides a Tkinter GUI to control the leader turtle.  
- Publishes velocity commands to `/turtle1/cmd_vel`.  
- Calls the `Spawn` service to create followers at random positions.  
- Publishes the names of new followers on `/new_follower`.

### 2. followers_node

- Subscribes to `/new_follower` to detect spawned turtles.  
- Subscribes to the leader and followers’ poses.  
- Computes velocity commands for each follower to follow the leader while maintaining a safe distance.  

### 3. turtlesim_node

- Provides the graphical turtlesim window and simulates turtle movements.

---





## Other Nodes

This package also includes several auxiliary nodes for demonstration and testing:

1. **first_node**
   - Publishes a counter message every second to the console.
   - **Purpose:** Simple example of a ROS 2 timer and logging.
   - **Run:**  
     ```bash
     ros2 run my_robot_controller test_node
     ```

2. **draw_circle**
   - Continuously publishes velocity commands to make the turtle move in a circle.
   - **Purpose:** Demonstrates publishing `Twist` messages for motion control.
   - **Run:**  
     ```bash
     ros2 run my_robot_controller draw_circle
     ```

3. **pose_subscriber**
   - Subscribes to `/turtle1/pose` and logs the turtle’s (x, y) position.
   - **Purpose:** Demonstrates subscribing to `Pose` messages.
   - **Run:**  
     ```bash
     ros2 run my_robot_controller pose_subscriber
     ```

4. **turtle_controller**
   - Subscribes to the turtle’s pose and publishes velocity commands.
   - Changes pen color when crossing a specific x-coordinate.
   - **Purpose:** Demonstrates combining subscriptions, publishers, and service calls.
   - **Run:**  
     ```bash
     ros2 run my_robot_controller turtle_controller
     ```

---

These nodes can be run individually using `ros2 run my_robot_controller <node_name>` for testing or experimentation.






## Installation & Usage
Step 0: Create a ROS 2 Workspace

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

Clone or copy this package into the src folder.

### Step 1: Run the Docker container

sudo docker run -it --net=host --device=/dev/dri:/dev/dri
--env="DISPLAY" --env="QT_X11_NO_MITSHM=1"
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
--name=humble osrf/ros:humble-desktop bash -it

Uses the official ROS 2 Humble desktop image.
Enables GUI forwarding for Tkinter and turtlesim.
Mounts the X11 socket to allow graphics.

Credits: https://github.com/shnx/robot_docker


### Step 2: Add a host alias for easy container startup

1. Edit your host ~/.bashrc:

gedit ~/.bashrc

Add:

alias humble='xhost + && sudo docker start ROS2_Humble && sudo docker exec -it ROS2_Humble bash'

Purpose:

xhost + allows GUI applications inside the container to display on your host.

docker start ROS2_Humble starts the container if it isn’t running.

docker exec -it ROS2_Humble bash opens an interactive shell inside the container.

2. Reload bash configuration:

source ~/.bashrc

3. Start your ROS 2 environment:

humble

### Step 3: Configure the ROS 2 environment inside the container

1. Edit the container’s ~/.bashrc and add:

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

source /opt/ros/humble/setup.bash: sets up ROS 2 environment variables for all ROS commands.

export ROS_DOMAIN_ID=0: ensures all nodes communicate on the same ROS 2 domain.

2. Reload bash configuration in container:

source ~/.bashrc

### Step 4: Build and run the package

1. **Clone the repository into your workspace’s `src` folder** 

cd ~/ros2_ws/src
git clone <repo-url>

2. Navigate to the workspace root:

cd ~/ros2_ws

3. Build the package using colcon:

colcon build --packages-select my_robot_controller

4. Source the workspace to overlay the newly built package:

source install/setup.bash

5. Launch the system:

ros2 launch my_robot_controller follow_turtles.launch.py

-The turtlesim window opens.

-The leader GUI opens for interactive control.

Use "Spawn Follower" to spawn turtles that automatically follow the leader.

=======
# ros2-turtle-follower
"A ROS 2 Humble package for controlling a leader turtle with Tkinter GUI and automatically spawning followers in turtlesim."
>>>>>>> d124ebbe0e424e9fcd6edc33714007e2c2f9777e
