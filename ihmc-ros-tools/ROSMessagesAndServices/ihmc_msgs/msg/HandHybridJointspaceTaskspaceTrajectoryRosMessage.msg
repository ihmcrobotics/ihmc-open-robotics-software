## HandHybridJointspaceTaskspaceTrajectoryRosMessage
# This message commands the controller to move the chest in both taskspace amd jointspace to the
# desired orientation and joint angles while going through the specified trajectory points.

# Specifies the side of the robot that will execute the trajectory.
int8 robot_side

# The taskspace trajectory information.
ihmc_msgs/SE3TrajectoryRosMessage taskspace_trajectory_message

# The jointspace trajectory information.
ihmc_msgs/JointspaceTrajectoryRosMessage jointspace_trajectory_message

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


