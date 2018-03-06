# This message is part of the IHMC whole-body controller API.
# Request the controller to use a hand to help supporting the robot weight.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# The robot side of the hand that will be load bearing.
byte robot_side 255
# Determines whether hybrid load bearing and jointspace control will be used.
bool use_jointspace_command false
# The arm desired jointspace trajectory that will be used for hybrid control if use_jointspace_command is true.
# The indexing for the joints goes increasingly from the first shoulder joint to the last arm joint.
JointspaceTrajectoryMessage jointspace_trajectory
# The time to delay this message on the controller side before being executed.
float64 execution_delay_time 0
# Information specific to the load bearing properties.
LoadBearingMessage load_bearing_message