# Interact Programmatically with Nav2
So far we've done using Nav2 to set the initial pose and then send some navigation goals. Now interact programmatically with Nav2 so that integrate the Nav2 functionallities directly in own code and inside the code and inside ROS2 nodes.

* Set an initial pose in Rviz: Using a topic.
* Send a navigation goal: Using an action.

In Nav2, already have an interface. Topics pubs, service servers, action servers are used and then interact with them from ROS2 code.