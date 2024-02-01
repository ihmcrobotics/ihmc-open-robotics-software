# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move a leg in jointspace to the desired joint angles while going through the specified trajectory points.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the side of the robot that will execute the trajectory.
byte robot_side 255
# Trajectories for each joint.
# The indexing for the joints goes increasingly from the first hip joint to the last leg joint.
JointspaceTrajectoryMessage jointspace_trajectory