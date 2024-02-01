# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Specifies the side of the robot that this message refers to.
byte robot_side 255

# Preset arm configuration
int32 preset

# Joint angles
float64[7] joint_angles

# Duration of the trajectory
float64 trajectory_duration
