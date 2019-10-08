# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Trajectory for the left hand
controller_msgs/HandTrajectoryMessage left_hand_trajectory_message

# Trajectory for the right hand
controller_msgs/HandTrajectoryMessage right_hand_trajectory_message

# Trajectory for the left arm joints
controller_msgs/ArmTrajectoryMessage left_arm_trajectory_message

# Trajectory for the right arm joints
controller_msgs/ArmTrajectoryMessage right_arm_trajectory_message

# Trajectory for the chest
controller_msgs/ChestTrajectoryMessage chest_trajectory_message

# Trajectory for the spine joints
controller_msgs/SpineTrajectoryMessage spine_trajectory_message

# Trajectory for the pelvis
controller_msgs/PelvisTrajectoryMessage pelvis_trajectory_message

# Trajectory for the left foot
controller_msgs/FootTrajectoryMessage left_foot_trajectory_message

# Trajectory for the right foot
controller_msgs/FootTrajectoryMessage right_foot_trajectory_message

# Trajectory for the neck joints
controller_msgs/NeckTrajectoryMessage neck_trajectory_message

# Trajectory for the head
controller_msgs/HeadTrajectoryMessage head_trajectory_message


