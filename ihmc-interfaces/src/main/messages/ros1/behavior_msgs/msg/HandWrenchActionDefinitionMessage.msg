
# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Specifies the side of the robot that this message refers to.
# Field default value 255
int8 robot_side

# Name of the frame the this action is expressed in
string parent_frame_name

# The trajectory duration
float64 trajectory_duration

# Amount of force to use along X
float64 force_x

# Amount of force to use along Y
float64 force_y

# Amount of force to use along Z
float64 force_z

# Amount of torque to use along X
float64 torque_x

# Amount of torque to use along Y
float64 torque_y

# Amount of torque to use along Z
float64 torque_z


