## HandPosePacketMessage
# No Documentation Annotation Found

#Options for enum# uint8 L = 0
# uint8 R = 1
uint8 robotSide
#Options for enum# uint8 CHEST = 0
# uint8 WORLD = 1
uint8 referenceFrame
#Options for enum# uint8 HAND_POSE = 0
# uint8 JOINT_ANGLES = 1
uint8 dataType
bool toHomePosition
geometry_msgs/Vector3 position
geometry_msgs/Quaternion orientation
float64 trajectoryTime
float64[] jointAngles

