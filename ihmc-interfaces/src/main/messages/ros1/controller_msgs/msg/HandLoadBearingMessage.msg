int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The robot side of the hand that will be load bearing.
# Field default value 255
int8 robot_side

# Determines whether hybrid load bearing and jointspace control will be used.
bool use_jointspace_command

# The arm desired jointspace trajectory that will be used for hybrid control if use_jointspace_command is true.
# The indexing for the joints goes increasingly from the first shoulder joint to the last arm joint.
controller_msgs/JointspaceTrajectoryMessage jointspace_trajectory

# The time to delay this message on the controller side before being executed.
float64 execution_delay_time

# Information specific to the load bearing properties.
controller_msgs/LoadBearingMessage load_bearing_message


