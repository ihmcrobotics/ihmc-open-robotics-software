# This message is part of the IHMC whole-body controller API.
# General purpose message that holds onto information to execute a trajectory in jointspace.
# A third order polynomial function is used to interpolate between trajectory points.
# The joint_trajectory_messages can have different waypoint times and different number of waypoints.
# If a joint trajectory message is empty, the controller will hold the last desired joint position while executing the other joint trajectories.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Trajectory for each joint.
controller_msgs/OneDoFJointTrajectoryMessage[] joint_trajectory_messages

# Properties for queueing trajectories.
ihmc_common_msgs/QueueableMessage queueing_properties


