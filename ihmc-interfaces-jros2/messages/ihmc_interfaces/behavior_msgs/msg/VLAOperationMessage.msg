# A message for remotely operating the VLA policies

ihmc_common_msgs/LatestModificationMessage latest_timestamp_modifiable

# Monotonically increasing, number of updates of robot thread
uint32 sequence_id

# Whether the inference and action output to IK streaming is active
bool running

# Whether to enable controlling the robot instead of just previewing
bool control_robot

# The latest action hand poses
geometry_msgs/Pose[2] action_hand_poses

# The latest action forearm poses
geometry_msgs/Pose[2] action_forearm_poses

# Status message from robot thread to show to the operator
string status_message
