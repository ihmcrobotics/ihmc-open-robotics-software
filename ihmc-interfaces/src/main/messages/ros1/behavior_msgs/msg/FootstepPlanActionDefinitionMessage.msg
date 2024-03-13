
# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Name of the parent frame the footsteps are expressed in
string parent_frame_name

# Swing duration
float64 swing_duration

# Transfer duration
float64 transfer_duration

# Is using the footstep planner or manually placing
bool is_manually_placed

# The footsteps, with a maximum of 50
behavior_msgs/FootstepPlanActionFootstepDefinitionMessage[] footsteps

# Planning goal pose X in the parent frame
float64 goal_x_in_parent

# Planning goal pose Y in the parent frame
float64 goal_y_in_parent

# Planning goal pose Yaw in the parent frame
float64 goal_yaw_in_parent

# Left goal foot X to the goal pose
float64 left_goal_foot_x_to_gizmo

# Left goal foot Y to the goal pose
float64 left_goal_foot_y_to_gizmo

# Left goal foot Yaw to the goal pose
float64 left_goal_foot_yaw_to_gizmo

# Right goal foot X to the goal pose
float64 right_goal_foot_x_to_gizmo

# Right goal foot Y to the goal pose
float64 right_goal_foot_y_to_gizmo

# Right goal foot Yaw to the goal pose
float64 right_goal_foot_yaw_to_gizmo


