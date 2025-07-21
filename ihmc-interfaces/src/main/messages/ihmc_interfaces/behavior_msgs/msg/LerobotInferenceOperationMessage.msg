# A message for remotely operationg the lerobot policies

# Allows the user to operate the inference of visuomotor policies
ihmc_common_msgs/LatestModificationMessage latest_timestamp_modifiable

# Monotonically increasing, number of updates of robot thread
uint32 sequence_id

# Whether the inference and action output to IK streaming is active
bool running

# Whether to enable controlling the robot instead of just previewing
bool control_robot

# Frequency of status from python side
float64 python_status_frequency

# Number of output actions received from policy
uint32 received_actions
