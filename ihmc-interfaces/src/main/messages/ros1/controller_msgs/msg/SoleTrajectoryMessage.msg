int8 FRONT_LEFT=0

int8 FRONT_RIGHT=1

int8 HIND_RIGHT=2

int8 HIND_LEFT=3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies which foot will execute the trajectory.
# Field default value 255
int8 robot_quadrant

# The position trajectory information.
controller_msgs/EuclideanTrajectoryMessage position_trajectory


