# This message is part of the IHMC whole-body controller API.
# Request the controller to use a hand to help supporting the robot weight.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The robot side of the hand that will be load bearing.
byte robot_side 255
# Determines the control mode used for the arm's nullspace. If true, jointspace is used, otherwise taskspace orientation is used.
bool use_jointspace_command true
# The arm desired jointspace trajectory that will be used to control the arm's nullspace if use_jointspace_command is true.
# The indexing for the joints goes increasingly from the first shoulder joint to the last arm joint.
JointspaceTrajectoryMessage jointspace_trajectory
# The hand desired orientation trajectory that will be used to control the arm's nullspace if use_jointspace_command is false.
ihmc_common_msgs/SO3TrajectoryMessage orientation_trajectory
# The time to delay this message on the controller side before being executed.
float64 execution_delay_time 0
# Information specific to the load bearing properties.
LoadBearingMessage load_bearing_message