## HandPosePacketMessage
# This message commands the controller to move an arm end effector to a given
# position and orientation. On Atlas, the end effector position is considered
# as the end of the hand attachment plate, which is about 10 cm from the end
# of the wrist. The position/orientation may be specified in world frame, chest
# frame, or desired joint angles.

# Options for robotSide
# uint8 L = 0
# uint8 R = 1
uint8 robot_side

# Options for dataType
# uint8 HAND_POSE = 0
# uint8 JOINT_ANGLES = 1
uint8 data_type

# Options for referenceFrame
# uint8 CHEST = 0
# uint8 WORLD = 1
uint8 reference_frame

# toHomePosition can be used to move the arm end effectors back to their starting
# position, defined as down and beside the robot with slightly bent elbows
bool to_home_position

geometry_msgs/Vector3 position

geometry_msgs/Quaternion orientation

# trajectoryTime specifies how fast or how slow to move to the desired pose
float64 trajectory_time

# jointAngles specifies the desired arm joint angles in order. For Atlas the
# controller assumes joint angles will be given in the follow order: shoulder
# pitch, shoulder roll, elbow pitch, elbow roll, wrist pitch, wrist roll
float64[] joint_angles


