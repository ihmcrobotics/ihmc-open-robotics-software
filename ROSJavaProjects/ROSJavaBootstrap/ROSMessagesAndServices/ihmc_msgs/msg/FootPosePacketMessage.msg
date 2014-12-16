## FootPosePacketMessage
# This message gives the position and oreientation of one foot in world coordinates.

#Options for enum
# uint8 L = 0
# uint8 R = 1
uint8 robotSide
geometry_msgs/Vector3 position
geometry_msgs/Quaternion orientation
float64 trajectoryTime

