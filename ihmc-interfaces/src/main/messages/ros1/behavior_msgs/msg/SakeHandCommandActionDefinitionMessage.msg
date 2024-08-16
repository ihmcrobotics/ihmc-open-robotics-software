
# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Specifies the side of the robot that this message refers to.
# Field default value 255
int8 robot_side

# Desired angle between the fingers from 0 to 210 degrees (specified in radians)
float64 hand_open_angle

# If hand angles are less than this amount off, skip the action
float64 initial_satisfaction_hand_angle_tolerance

# Hand angles goal error tolerance
float64 completion_hand_angle_tolerance

# Torque limit specified as fingertip grip force. Valid up to 29 Newtons. Safe value is 8.7 N.
float64 fingertip_grip_force_limit


