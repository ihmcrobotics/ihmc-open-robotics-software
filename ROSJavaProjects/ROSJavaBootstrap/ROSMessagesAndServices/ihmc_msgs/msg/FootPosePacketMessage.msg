## FootPosePacketMessage
# This message commands the controller to move the desired foot to a given location
# and orientation in world coordinates. This message is not intended to specify footsteps
# as the controller will use a straight-line trajectory when moving the foot from its
# current to desired location/orientation

#Options for robotSide
# uint8 L = 0
# uint8 R = 1
uint8 robotSide

geometry_msgs/Vector3 position

geometry_msgs/Quaternion orientation

# trajectoryTime specifies how fast or how slow to move to the desired pose
float64 trajectoryTime

int8 destination


