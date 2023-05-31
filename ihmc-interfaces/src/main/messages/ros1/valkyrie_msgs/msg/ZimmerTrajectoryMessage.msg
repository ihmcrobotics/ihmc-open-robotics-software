
# This message is Valkyrie specific and is to be used with a Zimmer gripper.
# This message commands the finger controller to move the robot fingers.
int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

int8 FINGER_1=0

int8 FINGER_2=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the side of the robot that will execute the trajectory.
# Field default value 255
int8 robot_side

# Specifies the list of joints that we want to control.
int8[] joint_names

# The indexing for the joints refers the list of joint names.
controller_msgs/JointspaceTrajectoryMessage jointspace_trajectory


