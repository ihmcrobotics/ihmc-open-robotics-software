# Behavior System Examples
These examples should provide insight on how to communicate with the robot (hardware, or simulation) over IHMC's ROS2 interfaces.
This directory can be treated as a colcon workspace. 

Use
```
$ ./compile_interfaces.sh
```
to compile the IHMC ROS2 interfaces into the current colcon workspace (this directory).

After compiling the IHMC ROS2 interfaces, run
```
$ source install/setup.bash
```

Ensure you've set `ROS_DOMAIN_ID` equal to `RTPSDomainID` in `~/.ihmc/IHMCNetworkParameters.ini`. If
you have not created or edited this file, run `NetworkParametersCreator` and set `RTPSDomainID` and
`RTPSSubnet`.
```
$ export ROS_DOMAIN_ID=99 # Change to the domain ID you've set in IHMCNetworkParameters.ini
```

## Running examples
After setting up the colcon workspace and launching `<robotName>RDXBehaviorSimulationUI`, you are ready to
test that everything's working correctly.

Requirements (install via pip):
- rclpy
- opencv-python

In the UI, locate the Behavior Panel and click on AI2RDemo.json to load a predefined list of behaviors.
Then run the module that hosts the reasoning for coordinating the behaviors:
```
$ python3 behaviors_example.py
```
Edit the file to coordinate the robot behaviors according to your custom reasoning/heuristics

You can also test the scene graph only by running
```
$ python3 scene_graph_example.py
```
Observe that some scene nodes are added in the scene graph and in the scene and removed.
