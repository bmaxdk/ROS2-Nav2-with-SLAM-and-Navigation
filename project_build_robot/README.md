<h1 align="center">Adapting a Custom Robot for Nav2 Project</h1>

Start with `TF transforms` that are required for the navigation to stack. When you launch `rviz` with TurtleBot3, you have seen that we can dispay the `TF transforms` for the robot.

<img src="image/a1.png">

In `TF`, there is different frames, `base_link`, `base_footprint`, `base_scan`, etc..  

In ROS2 we need to keep track of each frame of the robot and the environment relative to all other frame. You could need to know where a robot is relative to another robot or where a laser scan is relative to the map origin or to another part of the robots, etc.

The default solution to find this is to compute a `3D translation` plus `3D rotation` between each existing frame. Using `TF` package allows this process simpler. `TF package` which is named `TF` to the `TF package` will keep track of each 3D coordinate frame over time and it is as true to the tree of all the frames in your robot and in the environemnt.

<img src="image/a2.png">

TFs needed for Nav2:
1. `map` -> `odom`
2. `odom` -> `base_link`
3. `base_link` -> `base_scan`










# Additional Useful Sources 
[TF](https://husarion.com/tutorials/ros-tutorials/6-transformation-in-ROS/)