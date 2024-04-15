
# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Name of the parent frame the footsteps are expressed in
string parent_frame_name

# Specifies the side of the robot that will execute the kick.
# Field default value 255
int8 robot_side

# The height at which the kick should be targeted.
float64 kick_height

# The impulse with which the kick should be executed.
float64 kick_impulse

# The target distance from the robot to where the kick should be aimed.
float64 kick_target_distance

# The stance foot width.
float64 stance_foot_width

# Weight distribution before the kick. 1.0 means all weight on the kicking foot. Default is 0.5.
float64 prekick_weight_distribution


