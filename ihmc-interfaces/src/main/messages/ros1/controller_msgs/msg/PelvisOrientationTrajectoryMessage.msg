# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Whether the pelvis orientation is allowed to be controlled by the user when the robot is walking.
bool enable_user_pelvis_control_during_walking

# The orientation trajectory information.
controller_msgs/SO3TrajectoryMessage so3_trajectory


