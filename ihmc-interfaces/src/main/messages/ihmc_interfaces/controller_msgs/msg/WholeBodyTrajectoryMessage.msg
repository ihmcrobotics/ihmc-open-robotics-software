# This message is part of the IHMC whole-body controller API.
# Send whole body trajectories to the robot. A best effort is made to execute the trajectory while balance is kept.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Trajectory for the left hand
HandTrajectoryMessage left_hand_trajectory_message
# Trajectory for the right hand
HandTrajectoryMessage right_hand_trajectory_message

# Trajectory for the left arm joints
ArmTrajectoryMessage left_arm_trajectory_message
# Trajectory for the right arm joints
ArmTrajectoryMessage right_arm_trajectory_message

# Trajectory for the chest
ChestTrajectoryMessage chest_trajectory_message

# Trajectory for the spine joints
SpineTrajectoryMessage spine_trajectory_message

# Trajectory for the pelvis
PelvisTrajectoryMessage pelvis_trajectory_message

# Trajectory for the left foot
FootTrajectoryMessage left_foot_trajectory_message
# Trajectory for the right foot
FootTrajectoryMessage right_foot_trajectory_message

# Trajectory for the neck joints
NeckTrajectoryMessage neck_trajectory_message
# Trajectory for the head
HeadTrajectoryMessage head_trajectory_message