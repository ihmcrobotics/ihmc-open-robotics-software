## FootstepDataMessage
# This message specifies the position, orientation and side (left or right) of a desired footstep in
# world frame

#Options for robotSide
# uint8 L = 0
# uint8 R = 1
uint8 robotSide

geometry_msgs/Vector3 location

geometry_msgs/Quaternion orientation

# predictedContactPoints gives the vertices of the expected contact polygon between the foot and
# the world. A value of null will use the default controller contact points
Point2dMessage[] predictedContactPoints


