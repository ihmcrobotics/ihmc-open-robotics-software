
# This message is Valkyrie specific and is to be used with a Psyonic hand.
# This message commands the finger controller to move the robot fingers.
int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

int8 THUMB_Q1=0

int8 THUMB_Q2=1

int8 INDEX_Q1=2

int8 INDEX_Q2=3

int8 MIDDLE_Q1=4

int8 MIDDLE_Q2=5

int8 RING_Q1=6

int8 RING_Q2=7

int8 PINKY_Q1=8

int8 PINKY_Q2=9

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the side of the robot that will execute the trajectory.
# Field default value 255
int8 robot_side

# Specifies the list of joints that we want to control.
int8[] joint_names

# The indexing for the joints refers the list of joint names.
controller_msgs/JointspaceTrajectoryMessage jointspace_trajectory


