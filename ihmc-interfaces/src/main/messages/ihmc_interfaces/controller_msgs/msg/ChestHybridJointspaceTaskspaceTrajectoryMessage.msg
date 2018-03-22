# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move the chest in both taskspace and jointspace 
# to the desired orientation and joint angles while going through the specified trajectory points.

# The taskspace trajectory information.
SO3TrajectoryMessage taskspace_trajectory_message
# The jointspace trajectory information.
# The indexing for the joints goes increasingly from the joint the closest to the pelvis to the joint the closest to the chest.
JointspaceTrajectoryMessage jointspace_trajectory_message