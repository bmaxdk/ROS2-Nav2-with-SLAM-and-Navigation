# ROS2-Nav2-with-SLAM-and-Navigation
Ros2 Navigation 2 Stack

# Why Nav2?
Navigation2, or Nav2 for short, is the second generation of the ROS Navigation stack, designed specifically for ROS2. It offers a comprehensive solution for autonomous navigation in robotics and it is particularly useful and important for a number of reasons:

- Improved Performance: Nav2 takes full advantage of the features of ROS2. It's designed with a focus on multi-robot systems, safety, and mission critical reliability, utilizing the real-time and high-performance aspects of ROS2.

- Modularity: Nav2 has a modular design, enabling users to replace or augment different parts of the system to cater to their specific use-cases. For example, you could replace the path planner, controller, or the recovery behaviors with your own implementations.

- Behavior Trees: Nav2 uses behavior trees for task planning, which offer more flexibility and sophistication compared to the state machine model used in the original ROS Navigation stack. This allows more complex behaviors to be encapsulated in a maintainable and reusable way.

- Advanced Features: Nav2 introduces several advanced navigation features not available in the original ROS Navigation stack. For example, it supports dynamic obstacle avoidance, multi-map support, multi-robot support, and 3D perception.

- Safety: Safety and reliability are key priorities in Nav2. It supports safety plugins and various fallback behaviors to ensure safe operation in challenging environments.

- Active Community Support: Nav2 is actively maintained and used by a large community, ensuring regular updates, bug fixes, and improvements.

These attributes make Nav2 an essential tool for building advanced robotic systems using ROS2.

# Setup and Installation
**Version**
- ROS2 Foxy/Ubuntu 20.04

**Install the Nav2 package:**
```bash
$ source /opt/ros/foxy/setup.bash
$ sudo apt update
$ sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup
```

**Install the ROS2 Turtlebot3**
```bash
$ source /opt/ros/foxy/setup.bash

**[Option1]**
```bash
$ sudo apt install ros-foxy-turtlebot3*
```
**[Option2]**

# Create a new ROS2 workspace:
```bash
$ mkdir -p ~/turtlebot3_ws/src
$ cd ~/turtlebot3_ws
```
#Clone the TurtleBot3 packages:
```bash
$ cd ~/turtlebot3_ws/src/
$ git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
# Install Dependencies:
```bash
$ sudo apt install python3-colcon-common-extensions
$ sudo apt install terminator
```

# Generate a Map with SLAM
The goal for navigation is to make a robot move from one place to another while avoiding collision. 

**Navigation: a 2 step process**:
* **Step1:** Create a map (with SLAM) - Fist create a map of the world(the space where the robot can move).
* **Step2:** Make the robot navigate from point A to point B

**SLAM**: Allows the robot to localize itself in the environment relative to all of the worlds, the objects, obstacles and at the same time, it will also map this environemnt.

**Cehck Sections Below:**
* [grid based fast SLAM](https://github.com/bmaxdk/grid-based-fastSLAM)
* [graph SLAM](https://github.com/bmaxdk/graphSLAM)
* [SLAM RTAB Map](https://github.com/bmaxdk/RoboticsND-RTAB-Map-My-World)

# Turtlebot3 Environment Launch
```bash
$ source /opt/ros/foxy/setup.bash

# set turtlebot3 robot
$ export TURTLEBOT3_MODEL=waffle   
```
You can simply save this setup at `bashrc` 
```bash
$ vim ~/.bashrc
$ source .bashrc
$ printenv | grep TURTLE
```
**Control turtlebot3**
```bash
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Run telop key
$ ros2 run turtlebot3_teleop teleop_keyboard
```

# Generate and Save a Map with SLAM in ROS2
```bash
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Launch SLAM feature for turtlebot3 and set simulation time
$ ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
# This launch file contains SLAM feature

# Run telop key
$ ros2 run turtlebot3_teleop teleop_keyboard
```
### Move around to create map
```bash
# Save map into map folder
$ mkdir map

# Save map
$ ros2 run nav2_map_server map_saver_cli -f maps/my_map

```
