
# This message commands the finger controller to move robot valkyrie finger.
int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

int8 THUMB_MOTOR_ROLL=0

int8 THUMB_MOTOR_PITCH1=1

int8 THUMB_MOTOR_PITCH2=2

int8 INDEX_FINGER_MOTOR_PITCH1=3

int8 MIDDLE_FINGER_MOTOR_PITCH1=4

int8 PINKY_MOTOR_PITCH1=5

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the side of the robot that will execute the trajectory.
# Field default value 255
int8 robot_side

# Specifies the list of finger motor names that we want to control.
int8[] valkyrie_finger_motor_names

# The indexing for the joints refers the list of finger motor names.
controller_msgs/JointspaceTrajectoryMessage jointspace_trajectory


