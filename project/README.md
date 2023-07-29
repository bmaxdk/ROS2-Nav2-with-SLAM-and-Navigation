<h1 align="center">ROS2-Nav2 Project</h1>

# Build and Save a World in the Gazebo Building Editor
```bash
$ gazebo
```
`Edit << Building Editor` then begin building. 

Once world is create `File << Save As`. Now you will have **model.config** and **model.sdf** files.

Now start the Gazebo again
```bash
$ gazebo
```
In `Insert` pannel, you can import directly add your world into Gazebo.

## **[Optional] Using Floor Plan** 
If you have a floor plan image (in png), you can import from `Edit << Building Editor` then begin building. And then Scaled it well.

Now, the image with corrected scaled is shown. Use wall to create. For more precise use `shift` key.

## Build world file
```bash
$ gazebo
```
`Insert` pannel << insert the your model. Once it is doen, `File` << `Save World As` and then save as  `.world` format.

To open <name_of world file>.world:
```bash
$ gazebo my_world.world
```

Now we have a complete custom world based on a floor plan. This will simulated world will be used to testing.

# Make TurtleBot3 Navigate In the world
Now make turtlebot3 to navigate in this world. The goal would be to add up the robot so that it can move in the world. Also generate a map and navigate in the map.

[Optional] Here, pay attention this `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`. Understand lauch.py structure will be useful.
```bash
$ cd /opt/ros/foxy/share
```
Here in `share` folder have all of the packages. 
```bash
$ cd turtlebot3_gazebo 
$ cd worlds
```
Will indicates where examples of world files are located.

## Make turtlebot3 workspace
Build ROS2 workspace
```bash
$ mkdir -p turtlebot3_ws/src 
$ cd turtlebot3_ws/src
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# check out correct version. For example for foxy
$ git checkout foxy-devel
$ git pull
$ 
```



[Troubleshooting]
If gazebo is not launching, try:
```bash
$ source /usr/share/gazebo/setup.sh
```