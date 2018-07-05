# This message commands the finger controller to move robot valkyrie finger.
byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

byte THUMB_MOTOR_ROLL = 0
byte THUMB_MOTOR_PITCH1 = 1
byte THUMB_MOTOR_PITCH2 = 2
byte INDEX_FINGER_MOTOR_PITCH1 = 3
byte MIDDLE_FINGER_MOTOR_PITCH1 = 4
byte PINKY_MOTOR_PITCH1 = 5

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the side of the robot that will execute the trajectory.
byte robot_side 255

# Specifies the list of finger motor names that we want to control.
byte[] finger_motor_names

# The indexing for the joints refers the list of finger motor names.
JointspaceTrajectoryMessage jointspace_trajectory

# The list of times for trajectory execution delay.
float64[] delay_times