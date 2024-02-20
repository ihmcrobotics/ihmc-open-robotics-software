# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Specifies the side of the robot that this message refers to.
byte robot_side 255

# Desired angle between the fingers from 0 to 210 degrees (specified in radians)
float64 hand_open_angle

# Torque limit specified as fingertip grip force. Valid up to 29 Newtons. Safe value is 8.7 N.
float64 fingertip_grip_force_limit