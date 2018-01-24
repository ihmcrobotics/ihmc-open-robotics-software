## WholeBodyTrajectoryRosMessage
# Send whole body trajectories to the robot. A best effort is made to execute the trajectory while
# balance is kept.  A message with a unique id equals to 0 will be interpreted as invalid and will not
# be processed by the controller. This rule DOES apply to the fields of this message. If setting a
# field to null is not an option (going through IHMC ROS API), the user can use the latter rule to
# select the messages to be processed by the controller.

# Trajectory for the left hand
ihmc_msgs/HandTrajectoryRosMessage left_hand_trajectory_message

# Trajectory for the right hand
ihmc_msgs/HandTrajectoryRosMessage right_hand_trajectory_message

# Trajectory for the left arm joints
ihmc_msgs/ArmTrajectoryRosMessage left_arm_trajectory_message

# Trajectory for the right arm joints
ihmc_msgs/ArmTrajectoryRosMessage right_arm_trajectory_message

# Trajectory for the chest
ihmc_msgs/ChestTrajectoryRosMessage chest_trajectory_message

# Trajectory for the pelvis
ihmc_msgs/PelvisTrajectoryRosMessage pelvis_trajectory_message

# Trajectory for the left foot
ihmc_msgs/FootTrajectoryRosMessage left_foot_trajectory_message

# Trajectory for the right foot
ihmc_msgs/FootTrajectoryRosMessage right_foot_trajectory_message

# Trajectory for the head
ihmc_msgs/HeadTrajectoryRosMessage head_trajectory_message

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


