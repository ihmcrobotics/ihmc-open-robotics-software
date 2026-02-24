# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Name of the parent frame the footsteps are expressed in
string parent_frame_name

# Swing duration
float64 swing_duration

# Transfer duration
float64 transfer_duration

# OVERRIDE (0) or QUEUE (1)
byte execution_mode

# Is using the footstep planner or manually placing
bool is_manually_placed

# The footsteps, with a maximum of 50
behavior_msgs/FootstepPlanActionFootstepDefinitionMessage[<=50] footsteps

# The position to which the goal stance is aligned
geometry_msgs/Point goal_stance_point

# The point that the robot should be facing in the goal stance
geometry_msgs/Point goal_focal_point

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

# Initial stance side when planning
byte planner_initial_stance_side

# Whether to perform A* search in the planner
bool planner_perform_a_star_search

# Whether to use quick footstep planner
bool use_quick_footstep_planner

# Whether to walk, turning first, matching the goal orientation
bool planner_walk_with_goal_orientation

# Whether to use the body path planner or not
bool planner_plan_with_body_path

# The footstep planner parameters
ihmc_common_msgs/PrimitiveDataVectorMessage planner_parameters
