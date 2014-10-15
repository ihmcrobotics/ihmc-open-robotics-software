## HandPosePacketMessage
# This message commands the controller to move a hand to a given position
#  and orientation. The position/orientation may be specified in world
# frame, chest frame, or desired joint angles.

#Options for enum
# uint8 L = 0
# uint8 R = 1
uint8 robotSide
#Options for enum
# uint8 HAND_POSE = 0
# uint8 JOINT_ANGLES = 1
uint8 dataType
#Options for enum
# uint8 CHEST = 0
# uint8 WORLD = 1
uint8 referenceFrame
# toHomePosition can be used to move the hands back to their starting position: down and beside the robot
bool toHomePosition
geometry_msgs/Vector3 position
geometry_msgs/Quaternion orientation
float64 trajectoryTime
float64[] jointAngles

