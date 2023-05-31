# This message is Valkyrie specific and is to be used with a Psyonic hand.
# This message commands the finger controller to move the robot fingers.
byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

byte THUMB_Q1 = 0
byte THUMB_Q2 = 1
byte INDEX_Q1 = 2
byte INDEX_Q2 = 3
byte MIDDLE_Q1 = 4
byte MIDDLE_Q2 = 5
byte RING_Q1 = 6
byte RING_Q2 = 7
byte PINKY_Q1 = 8
byte PINKY_Q2 = 9

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the side of the robot that will execute the trajectory.
byte robot_side 255

# Specifies the list of joints that we want to control.
byte[<=10] joint_names

# The indexing for the joints refers the list of joint names.
controller_msgs/JointspaceTrajectoryMessage jointspace_trajectory