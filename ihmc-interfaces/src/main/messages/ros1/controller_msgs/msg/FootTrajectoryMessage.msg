int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies which foot will execute the trajectory.
# Field default value 255
int8 robot_side

# The position/orientation trajectory information.
controller_msgs/SE3TrajectoryMessage se3_trajectory


