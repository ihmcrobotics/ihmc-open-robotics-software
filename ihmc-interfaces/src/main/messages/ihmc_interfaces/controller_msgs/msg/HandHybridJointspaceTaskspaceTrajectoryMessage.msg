# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move the hand in both taskspace and jointspace 
# to the desired orientation and joint angles while going through the specified trajectory points.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When the robot is walking, restrictions on upper-body motion may be applied.
# To by-pass the safety check and force the execution of this message, set this field to true.
bool force_execution false

# Specifies the side of the robot that will execute the trajectory.
byte robot_side 255
# The taskspace trajectory information.
SE3TrajectoryMessage taskspace_trajectory_message
# The jointspace trajectory information.
# The indexing for the joints goes increasingly from the first shoulder joint to the last arm joint.
JointspaceTrajectoryMessage jointspace_trajectory_message