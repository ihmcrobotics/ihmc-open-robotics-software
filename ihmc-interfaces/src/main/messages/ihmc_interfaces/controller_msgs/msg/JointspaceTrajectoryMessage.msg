# This message is part of the IHMC whole-body controller API.
# General purpose message that holds onto information to execute a trajectory in jointspace.
# A third order polynomial function is used to interpolate between trajectory points.
# The joint_trajectory_messages can have different waypoint times and different number of waypoints.
# If a joint trajectory message is empty, the controller will hold the last desired joint position while executing the other joint trajectories.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

# Trajectory for each.
OneDoFJointTrajectoryMessage[] joint_trajectory_messages
# Properties for queueing trajectories.
QueueableMessage queueing_properties