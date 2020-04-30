# This message is part of the IHMC whole-body controller API.
# This message commands the controller to apply a wrench (force & moment) profile on a hand given a list of trajectory points.
# A linear interpolation is used to interpolate between trajectory points.

int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When the robot is walking, restrictions on upper-body motion may be applied.
# To by-pass the safety check and force the execution of this message, set this field to true.
bool force_execution

# Specifies which hand will execute the trajectory.
# Field default value 255
int8 robot_side

# The trajectory information for the force/moment to be achieved by the end-effector.
controller_msgs/WrenchTrajectoryMessage wrench_trajectory


