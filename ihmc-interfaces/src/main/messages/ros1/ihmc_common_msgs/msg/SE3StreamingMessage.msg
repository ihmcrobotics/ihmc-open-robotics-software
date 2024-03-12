# This is part of the IHMC Common message package.
# It is a lightweight version of SE3TrajectoryMessage designed for streaming.

ihmc_common_msgs/FrameInformation frame_information

# Flag that tells the controller whether the use of a custom control frame is requested.
bool use_custom_control_frame

# Pose of custom control frame expressed in the end-effector frame.
# This is the frame attached to the rigid body that the taskspace trajectory is defined for.
geometry_msgs/Pose control_frame_pose

# Define the desired 3D position to be reached.
geometry_msgs/Point position

# Define the desired 3D orientation to be reached.
geometry_msgs/Quaternion orientation

# Define the desired 3D linear velocity, expressed in world frame,  to be reached.
geometry_msgs/Vector3 linear_velocity

# Define the desired 3D angular velocity, expressed in world frame,  to be reached.
geometry_msgs/Vector3 angular_velocity

# Define the desired 3D linear acceleration, expressed in world frame,  to be reached.
geometry_msgs/Vector3 linear_acceleration

# Define the desired 3D angular acceleration, expressed in world frame,  to be reached.
geometry_msgs/Vector3 angular_acceleration


