
# Message used to report the computed footstep, which will be placed by the UI
int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# Specifies which foot
# Field default value 255
int8 robot_side

# Desired footstep position in world frame.
geometry_msgs/Point desired_foot_position

# Desired footstep orientation in world frame.
geometry_msgs/Quaternion desired_foot_orientation


