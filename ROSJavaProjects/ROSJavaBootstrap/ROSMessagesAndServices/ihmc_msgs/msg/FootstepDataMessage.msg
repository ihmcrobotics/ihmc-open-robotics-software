## FootstepDataMessage
# This message specifies the position, orientation and side (left or right) of a desired footstep in
# world frame

# Options for robotSide
# uint8 L = 0
# uint8 R = 1
uint8 robot_side

geometry_msgs/Vector3 location

geometry_msgs/Quaternion orientation

# This contains information on what the swing trajectory should be for each step
# DEFAULT gives the standard IHMC parabolic footstep trajectory
# LOW_HEIGHT gives a lower-arched trajectory
# STEP_ON_OR_OFF gives a higher-arched trajectory meant for stepping onto or off obstacles
# BY_BOX gives a square-wave type trajectory according to the parameters of trajectoryBoxData
# NO_STEP results in a null footstep. This probably should not be used outside of IHMC source
# Options for trajectoryType
# uint8 DEFAULT = 0
# uint8 BASIC = 1
# uint8 PUSH_RECOVERY = 2
# uint8 OBSTACLE_CLEARANCE = 3
uint8 trajectory_type

# Contains information on how high the robot should step
float64 swing_height


