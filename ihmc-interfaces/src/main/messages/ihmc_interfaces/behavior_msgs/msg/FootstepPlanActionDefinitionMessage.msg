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

# Quick footstep planner
byte QUICK = 0
# Turn walk turn planner
byte TURN_WALK_TURN = 1
# A star planner
byte A_STAR = 2

# The type of planner as defined above
byte planner

# Whether to walk, turning first, matching the goal orientation
bool planner_walk_with_goal_orientation

# Whether to use the body path planner or not
bool planner_plan_with_body_path

# The footstep planner parameters
ihmc_common_msgs/PrimitiveDataVectorMessage planner_parameters

# Quick footstep planner parameters
float64 quick_hip_width
float64 quick_step_length
float64 quick_next_pelvis_yaw_limit
float64 quick_inward_limit
float64 quick_outward_limit
float64 quick_step_angle_limit
float64 quick_swing_time_distance_lower
float64 quick_swing_time_distance_upper
float64 quick_min_swing_time
float64 quick_max_swing_time
