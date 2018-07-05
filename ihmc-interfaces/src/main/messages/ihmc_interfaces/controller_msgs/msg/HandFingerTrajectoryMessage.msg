# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move fingers in motor control space.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the side of the robot that will execute the trajectory.
byte robot_side 255

# Trajectories for each joint(finger motors).
OneDoFJointTrajectoryMessage[] joint_trajectory_messages
# Properties for queueing trajectories.
QueueableMessage[] list_queueing_properties