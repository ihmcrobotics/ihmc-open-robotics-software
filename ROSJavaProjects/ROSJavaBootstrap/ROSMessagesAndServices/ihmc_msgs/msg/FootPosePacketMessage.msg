## FootPosePacketMessage
# This message commands the controller to move the desired foot to a given location
# and orientation in world coordinates. This message is not intended to specify footsteps
# as the controller will use a straight-line trajectory when moving the foot from its
# current to desired location/orientation

# Options for robotSide
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot
uint8 robot_side

geometry_msgs/Vector3 position

geometry_msgs/Quaternion orientation

# trajectoryTime specifies how fast or how slow to move to the desired pose
float64 trajectory_time


