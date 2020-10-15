# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move in taskspace a hand to the desired pose (position & orientation) while going through the specified trajectory points.
# A third order polynomial function is used to interpolate positions and a hermite based curve (third order) is used to interpolate the orientations.
# To execute a single straight line trajectory to reach a desired hand pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.

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

# The position/orientation trajectory information.
controller_msgs/SE3TrajectoryMessage se3_trajectory


