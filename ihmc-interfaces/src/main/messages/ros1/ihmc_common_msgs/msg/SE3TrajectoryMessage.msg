# This is part of the IHMC Common message package.
# This message carries the information to execute a trajectory in taskspace (position and orientation) by defining trajectory points.
# A third order polynomial function is used to interpolate positions and a Hermite based curve (third order) is used to interpolate the orientations.
# To execute a single straight line trajectory to reach a desired pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# List of trajectory points (in taskpsace) to go through while executing the trajectory.
ihmc_common_msgs/SE3TrajectoryPointMessage[] taskspace_trajectory_points

# The selection matrix for each axis of the angular part of this trajectory.
ihmc_common_msgs/SelectionMatrix3DMessage angular_selection_matrix

# The selection matrix for each axis of the linear part of this trajectory.
ihmc_common_msgs/SelectionMatrix3DMessage linear_selection_matrix

# Frame information for this message.
ihmc_common_msgs/FrameInformation frame_information

# The weight matrix for each axis of the angular part of this trajectory.
ihmc_common_msgs/WeightMatrix3DMessage angular_weight_matrix

# The weight matrix for each axis of the linear part of this trajectory.
ihmc_common_msgs/WeightMatrix3DMessage linear_weight_matrix

# Flag that tells the controller whether the use of a custom control frame is requested.
bool use_custom_control_frame

# Pose of custom control frame expressed in the end-effector frame.
# This is the frame attached to the rigid body that the taskspace trajectory is defined for.
geometry_msgs/Pose control_frame_pose

# Properties for queueing trajectories.
ihmc_common_msgs/QueueableMessage queueing_properties


