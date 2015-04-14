## FootstepDataMessage
# This message specifies the position, orientation and side (left or right) of a desired footstep in
# world frame

# Options for robotSide
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot
uint8 robot_side

geometry_msgs/Vector3 location

geometry_msgs/Quaternion orientation

# This contains information on what the swing trajectory should be for each step
# Options for trajectoryType
uint8 DEFAULT=0 # is a default trajectory
uint8 BASIC=1 # will do a basic swing with the specified swing height
uint8 PUSH_RECOVERY=2 # uses a low swing height for fast steps
uint8 OBSTACLE_CLEARANCE=3 # will attempt to step over an obstacle
uint8 trajectory_type

# Contains information on how high the robot should step
float64 swing_height


