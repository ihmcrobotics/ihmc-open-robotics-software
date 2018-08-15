# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# List of trajectory points (in taskpsace) to go through while executing the trajectory.
controller_msgs/EuclideanTrajectoryPointMessage[] taskspace_trajectory_points

# The selection matrix for each axis.
controller_msgs/SelectionMatrix3DMessage selection_matrix

# Frame information for this message.
controller_msgs/FrameInformation frame_information

# The weight matrix for each axis.
controller_msgs/WeightMatrix3DMessage weight_matrix

# Flag that tells the controller whether the use of a custom control frame is requested.
bool use_custom_control_frame

# Pose of custom control frame. This is the frame attached to the rigid body that the taskspace trajectory is defined for.
geometry_msgs/Pose control_frame_pose

# Properties for queueing trajectories.
controller_msgs/QueueableMessage queueing_properties


