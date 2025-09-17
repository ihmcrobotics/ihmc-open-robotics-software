# A message for remotely operating the visuomotor policies

ihmc_common_msgs/LatestModificationMessage latest_timestamp_modifiable

# Monotonically increasing, number of updates of robot thread
uint32 sequence_id

# Whether the inference and action output to IK streaming is active
bool running

# Whether to enable controlling the robot instead of just previewing
bool control_robot

# The latest action hand poses
geometry_msgs/Pose[] action_hand_poses

# The latest action forearm poses
geometry_msgs/Pose[] action_forearm_poses

# Frequency of status from python side
float64 python_status_frequency

# Message received from python
string python_status_message

# Number of output actions received from policy
uint32 received_actions


