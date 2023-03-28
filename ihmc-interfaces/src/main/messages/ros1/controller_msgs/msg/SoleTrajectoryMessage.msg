# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move in taskspace a foot to the desired position  while going through the specified trajectory points.
# To execute a single straight line trajectory to reach a desired foot pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.

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
ihmc_common_msgs/EuclideanTrajectoryMessage position_trajectory


