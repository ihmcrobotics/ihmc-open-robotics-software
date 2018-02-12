## SE3TrajectoryRosMessage
# 

# List of trajectory points (in taskpsace) to go through while executing the trajectory. All the
# information contained in these trajectory points needs to be expressed in world frame.
ihmc_msgs/SE3TrajectoryPointRosMessage[] taskspace_trajectory_points

# The selection matrix for each axis of the angular part.
ihmc_msgs/SelectionMatrix3DRosMessage angular_selection_matrix

# The selection matrix for each axis of the linear part.
ihmc_msgs/SelectionMatrix3DRosMessage linear_selection_matrix

# Frame information for this message.
ihmc_msgs/FrameInformationRosMessage frame_information

# The weight matrix for each axis of the angular part.
ihmc_msgs/WeightMatrix3DRosMessage angular_weight_matrix

# The weight matrix for each axis of the linear part.
ihmc_msgs/WeightMatrix3DRosMessage linear_weight_matrix

# Flag that tells the controller whether the use of a custom control frame is requested.
bool use_custom_control_frame

# Pose of custom control frame. This is the frame attached to the rigid body that the taskspace
# trajectory is defined for.
geometry_msgs/Transform control_frame_pose

# Properties for queueing trajectories.
ihmc_msgs/QueueableRosMessage queueing_properties

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


