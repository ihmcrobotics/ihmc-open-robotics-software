# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The trajectories for each joint in order from the one closest to the chest to the one the closest to the head.
controller_msgs/JointspaceTrajectoryMessage jointspace_trajectory


