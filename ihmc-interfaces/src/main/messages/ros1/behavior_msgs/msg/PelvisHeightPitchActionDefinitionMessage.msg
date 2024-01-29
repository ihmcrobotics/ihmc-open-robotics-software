
# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Name of the frame the this action is expressed in
string parent_frame_name

# Transform that expresses the pelvis pose in the parent frame
controller_msgs/RigidBodyTransformMessage pelvis_transform_to_parent

# Duration of the trajectory
float64 trajectory_duration


