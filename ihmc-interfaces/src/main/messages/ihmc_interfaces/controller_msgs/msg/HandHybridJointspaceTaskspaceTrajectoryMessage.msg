# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move the hand in both taskspace and jointspace 
# to the desired orientation and joint angles while going through the specified trajectory points.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Specifies the side of the robot that will execute the trajectory.
byte robot_side -1
# The taskspace trajectory information.
SO3TrajectoryMessage taskspace_trajectory_message
# The jointspace trajectory information.
# The indexing for the joints goes increasingly from the first shoulder joint to the last arm joint.
JointspaceTrajectoryMessage jointspace_trajectory_message