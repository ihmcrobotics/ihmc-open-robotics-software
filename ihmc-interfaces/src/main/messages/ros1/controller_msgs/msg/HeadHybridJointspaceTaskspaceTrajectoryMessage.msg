# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The taskspace trajectory information.
controller_msgs/SO3TrajectoryMessage taskspace_trajectory_message

# The jointspace trajectory information.
# The indexing for the joints goes increasingly from the joint the closest to the chest to the joint the closest to the head.
controller_msgs/JointspaceTrajectoryMessage jointspace_trajectory_message


