# Nav2 Project

## Build and Save a World in the Gazebo Building Editor
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

# **[Optional]** 
If you have a floor plan image (in png), you can import from `Edit << Building Editor` then begin building. And then Scaled it well.

Now, the image with corrected scaled is shown. Use wall to create. For more precise use `shift` key.

# Build world file
```bash
$ gazebo
```
`Insert` pannel << insert the your model. Once it is doen, `File` << `Save World As` and then save as  `.world`.

To open my world:
```bash
$ gazebo my_world.world
```

Now we have a complete custom world based on a floor plan. This will simulated world will be used to testing.

# Make TurtleBot3 Navigate In the world
Now make turtlebot3 to navigate in this world. The goal would be to add up the robot so that it can move in the world. Also generate a map and navigate in the map.

Here, pay attention this `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`. We will modify world file
```bash
$ cd /opt/ros/foxy/share
```
Here have all of the packages.
```bash
$ cd turtlebot3_gazebo 
```
