# This message is part of the IHMC whole-body controller API.
# This message carries the information to execute a trajectory in taskspace (position only) by defining trajectory points.
# A third order polynomial function is used to interpolate positions.
# To execute a single straight line trajectory to reach a desired position, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# List of trajectory points (in taskpsace) to go through while executing the trajectory.
EuclideanTrajectoryPointMessage[] taskspace_trajectory_points
# The selection matrix for each axis.
SelectionMatrix3DMessage selection_matrix
# Frame information for this message.
FrameInformation frame_information
# The weight matrix for each axis.
WeightMatrix3DMessage weight_matrix
# Flag that tells the controller whether the use of a custom control frame is requested.
bool use_custom_control_frame
# Pose of custom control frame. This is the frame attached to the rigid body that the taskspace trajectory is defined for.
geometry_msgs/Pose control_frame_pose
# Properties for queueing trajectories.
QueueableMessage queueing_properties
