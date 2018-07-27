# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the leg kinematics.
bool enable_user_pelvis_control

# If enable_user_pelvis_control is true then enable_user_pelvis_control_during_walking
# will keep the manager in user mode while walking.
# If this is false the manager will switch back to controller mode when walking.
bool enable_user_pelvis_control_during_walking

# The position/orientation trajectory information.
controller_msgs/SE3TrajectoryMessage se3_trajectory


