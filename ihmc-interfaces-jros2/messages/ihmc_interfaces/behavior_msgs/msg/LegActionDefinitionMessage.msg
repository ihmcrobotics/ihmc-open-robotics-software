# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Specifies the side of the robot that this message refers to.
byte robot_side 255

# Name of the frame the this action is expressed in
string parent_frame_name

# Transform that expresses the hand pose in the parent frame
controller_msgs/RigidBodyTransformMessage transform_to_parent

# The trajectory duration
float64 trajectory_duration
