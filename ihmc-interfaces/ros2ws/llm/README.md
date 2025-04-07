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


## Errors while running examples
1. /usr/bin/python3.10 behaviors_example.py
Traceback (most recent call last):
  File "/home/sravani/nadia/repository-group/ihmc-open-robotics-software/ihmc-interfaces/ros2ws/behaviors_example.py", line 10, in <module>
    from behavior_msgs.msg import AI2RCommandMessage
ModuleNotFoundError: No module named 'behavior_msgs'

Solution: Run 
    source ~/nadia/repository-group/ihmc-open-robotics-software/ihmc-interfaces/ros2ws/install/setup.bash

Error: No output

Solution: export ROS_DOMAIN_ID=32

So Order is:
    1. source ~/nadia/repository-group/ihmc-open-robotics-software/ihmc-interfaces/ros2ws/install/setup.bash
    2. export ROS_DOMAIN_ID=32
    3. /usr/bin/python3.10 behaviors_example.py

2. API Key Calling
#os.environ["TOGETHER_API_KEY"] = ""

Do any of the below steps:
    1. export TOGETHER_API_KEY=""
    before running script 

    2. echo 'export TOGETHER_API_KEY=""' >> ~/.bashrc
       source ~/.bashrc

3. Current output

 Calling the LLM
Objects in the scene:
scene_objects:  [behavior_msgs.msg.AI2RObjectMessage(object_name='DoorLever1', object_pose_in_world=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=1.70865, y=2.41354, z=0.88848), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=-0.6919510436971168, w=0.721944425234014)), object_pose_in_robot_frame=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=-1.2498131093700098, y=1.589958399588408, z=1.0318632493409556), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=-0.7033781153636404, w=0.7108158881366492))), behavior_msgs.msg.AI2RObjectMessage(object_name='DoorPanel1', object_pose_in_world=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=2.03492, y=2.41911, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.7216816418936234, w=0.6922251134954033)), object_pose_in_robot_frame=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=-0.9235314578794593, y=1.5851188024358236, z=0.14338324934095564), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.710548766780828, w=0.7036479588730749))), behavior_msgs.msg.AI2RObjectMessage(object_name='Person1', object_pose_in_world=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=2.28347, y=-1.56887, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.8743219929352777, w=0.4853463224025543)), object_pose_in_robot_frame=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=-0.8023092073183835, y=-2.408759857321994, z=0.14338324934095564), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.86646944047566, w=0.4992301160004239))), behavior_msgs.msg.AI2RObjectMessage(object_name='Person2', object_pose_in_world=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=5.49537, y=-3.56887, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.8743219929352777, w=0.4853463224025543)), object_pose_in_robot_frame=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=2.344164200580669, y=-4.510189538130781, z=0.14338324934095564), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.86646944047566, w=0.4992301160004239))), behavior_msgs.msg.AI2RObjectMessage(object_name='Charge1', object_pose_in_world=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=1.66921, y=-1.70972, z=1.00789), orientation=geometry_msgs.msg.Quaternion(x=-0.013204441066068408, y=-0.0016826147121791343, z=0.9918906986491487, w=0.1263948316869275)), object_pose_in_robot_frame=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=-1.420749239716663, y=-2.529945650330972, z=1.1512732493409557), orientation=geometry_msgs.msg.Quaternion(x=-0.01322959920961659, y=-0.00147178833051073, z=0.9897485071733179, w=0.14219952212314854))), behavior_msgs.msg.AI2RObjectMessage(object_name='Barrier1', object_pose_in_world=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=2.3007, y=-0.68487, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.6844199937260306, w=0.7290879728729999)), object_pose_in_robot_frame=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=-0.7568917607336343, y=-1.5257592183382385, z=0.14338324934095564), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.6727038919177455, w=0.7399118013646748)))]
DoorLever1
DoorPanel1
Person1
Person2
Charge1
Barrier1
Available behaviors:
GOTO
SCAN
PICK UP CHARGE
PLACE CHARGE ON DOOR
Completed Behavior: GOTO
Commanded Behavior: GOTO

