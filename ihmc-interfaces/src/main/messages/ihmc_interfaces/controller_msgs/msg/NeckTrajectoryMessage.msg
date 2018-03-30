# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move the neck in jointspace to the desired joint angles while going through the specified trajectory points.
# A third order polynomial function is used to interpolate between trajectory points.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The trajectories for each joint in order from the one closest to the chest to the one the closest to the head.
JointspaceTrajectoryMessage jointspace_trajectory