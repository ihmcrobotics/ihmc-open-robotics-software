
# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Specifies the side of the robot that this message refers to.
# Field default value 255
int8 robot_side

# Hand configuration type
uint32 configuration

# Goal position as ratio from 0.0 (closed) to 1.0 (open)
float64 position_ratio

# Goal torque as ratio from 0.0 (closed) to 1.0 (open)
float64 torque_ratio


