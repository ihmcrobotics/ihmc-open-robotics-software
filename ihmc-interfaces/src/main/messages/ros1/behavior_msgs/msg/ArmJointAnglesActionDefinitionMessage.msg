
# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Specifies the side of the robot that this message refers to.
# Field default value 255
int8 robot_side

# Preset arm configuration
int32 preset

# Joint angles
float64[] joint_angles

# Duration of the trajectory
float64 trajectory_duration


