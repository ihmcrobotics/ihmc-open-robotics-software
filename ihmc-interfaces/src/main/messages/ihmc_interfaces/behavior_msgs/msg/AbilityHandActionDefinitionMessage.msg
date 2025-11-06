# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Specifies the side of the robot that this message refers to.
byte robot_side 255

# AbilityHandManager.ControlMode ordinal
int32 control_mode

# Grip to execute, if grip control mode
int32 grip

# Goal positions for the joints
float32[6] goal_positions

# Goal positions for the joints
float32[6] goal_velocities

