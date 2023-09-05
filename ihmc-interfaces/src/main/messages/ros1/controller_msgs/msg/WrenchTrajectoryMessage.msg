# This message is part of the IHMC whole-body controller API.
# This class is used to build a wrench (force & moment) profile over time.
# A linear interpolation is used to interpolate between trajectory points.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# List of trajectory points (in taskpsace) to go through while executing the trajectory.
controller_msgs/WrenchTrajectoryPointMessage[] wrench_trajectory_points

# Frame information for this message.
ihmc_common_msgs/FrameInformation frame_information

# Flag that tells the controller whether the use of a custom control frame is requested.
bool use_custom_control_frame

# Pose of custom control frame expressed in the end-effector frame.
# This is the frame attached to the rigid body that the wrench trajectory is defined for.
geometry_msgs/Pose control_frame_pose

# Properties for queueing trajectories.
ihmc_common_msgs/QueueableMessage queueing_properties


