# This message is part of the IHMC footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

byte FOOTSTEP_PLANNER_TYPE_PLAN_THEN_SNAP = 0
# The recommended planner type
byte FOOTSTEP_PLANNER_TYPE_A_STAR = 1
byte FOOTSTEP_PLANNER_TYPE_VIS_GRAPH_WITH_A_STAR = 2

int32 NO_PLAN_ID = -1

# Starting left foot pose
geometry_msgs/Pose start_left_foot_pose
# Starting right foot pose
geometry_msgs/Pose start_right_foot_pose
# Goal left foot pose
geometry_msgs/Pose goal_left_foot_pose
# Goal right foot pose
geometry_msgs/Pose goal_right_foot_pose
# Requested initial stance side. If not specified the planner will choose
byte requested_initial_stance_side 255
# If true, the planner will snap the provided goal steps. Otherwise the provided poses will be trusted as valid footholds.
bool snap_goal_steps true
# If snap_goal_steps is true and the goal steps can't be snapped, this specifies whether to abort or go ahead and plan.
bool abort_if_goal_step_snapping_fails

# Footstep planner type, see above
byte requested_footstep_planner_type 255
# Acceptable xy distance from the given goal for the planner to terminate
float64 goal_distance_proximity -1.0
# Acceptable yaw offset from the given goal for the planner to terminate
float64 goal_yaw_proximity -1.0
# Planner timeout in seconds. If max_iterations is set also, the planner terminates whenever either is reached
float64 timeout 5.0
# Maximum iterations. Set to a non-positive number to disable. If timeout is also set, the planner terminates whener either is reached.
int32 max_iterations -1
# Best effort timeout in seconds
float64 best_effort_timeout
# Max body path length if using body path
float64 horizon_length
# Planar regions to use, if you don't want to assume flat ground
PlanarRegionsListMessage planar_regions_list_message
# Explicitly tell the planner to use flat ground
bool assume_flat_ground
# Set this id to keep track of your request
int32 planner_request_id -1
# Requested body path waypoints. If non-empty, planner will follow this path and will not plan a body path
geometry_msgs/Pose[<=50] body_path_waypoints