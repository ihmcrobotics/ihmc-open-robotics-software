# This message is Valkyrie specific and is to be used with a Zimmer gripper.
# This message commands the finger controller to move the robot fingers.
byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

byte FINGER_1 = 0
byte FINGER_2 = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the side of the robot that will execute the trajectory.
byte robot_side 255

# Specifies the list of joints that we want to control.
byte[<=2] joint_names

# The indexing for the joints refers the list of joint names.
controller_msgs/JointspaceTrajectoryMessage jointspace_trajectory