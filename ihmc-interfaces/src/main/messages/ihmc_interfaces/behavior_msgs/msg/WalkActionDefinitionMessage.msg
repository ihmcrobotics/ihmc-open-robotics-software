# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Name of the frame the this action is expressed in
string parent_frame_name

# Transform that expresses the walk gizmo pose in the parent frame
controller_msgs/RigidBodyTransformMessage transform_to_parent

# Left goal foot transform to the walk gizmo
controller_msgs/RigidBodyTransformMessage left_goal_foot_transform_to_gizmo

# Right goal foot transform to the walk gizmo
controller_msgs/RigidBodyTransformMessage right_goal_foot_transform_to_gizmo

# Swing duration
float64 swing_duration

# Transfer duration
float64 transfer_duration