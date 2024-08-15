# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Name of the parent frame the footsteps are expressed in
string parent_frame_name

# Swing duration
float64 swing_duration

# Transfer duration
float64 transfer_duration

# OVERRIDE (0) or QUEUE (1)
int32 execution_mode

# Specifies the side of the robot that will execute the kick.
byte robot_side 255

# The impulse with which the kick should be executed.
float64 kick_height

# The impulse with which the kick should be executed.
float64 kick_impulse

# The target distance from the robot to where the kick should be aimed.
float64 kick_target_distance

# The distance towards the inside of the door from where the kick foot should be aligned.
float64 horizontal_distance_from_handle

# The stance foot width.
float64 stance_foot_width

# Weight distribution before the kick. 1.0 means all weight on the kicking foot. Default is 0.5.
float64 prekick_weight_distribution
