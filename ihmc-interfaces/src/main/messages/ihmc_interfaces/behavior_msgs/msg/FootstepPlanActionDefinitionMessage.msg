# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Name of the parent frame the footsteps are expressed in
string parent_frame_name

# Swing duration
float64 swing_duration

# Transfer duration
float64 transfer_duration

# The footsteps, with a maximum of 50
behavior_msgs/FootstepPlanActionFootstepDefinitionMessage[<=50] footsteps

# Transform that expresses the planning goal pose in the parent frame
controller_msgs/RigidBodyTransformMessage goal_transform_to_parent

# Left goal foot transform to the goal pose
controller_msgs/RigidBodyTransformMessage left_goal_foot_transform_to_gizmo

# Right goal foot transform to the goal pose
controller_msgs/RigidBodyTransformMessage right_goal_foot_transform_to_gizmo