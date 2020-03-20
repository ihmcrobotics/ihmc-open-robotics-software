# This message is part of the IHMC footstep planning module.

int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

int8 FOOTSTEP_PLANNER_TYPE_PLAN_THEN_SNAP=0

# The recommended planner type
int8 FOOTSTEP_PLANNER_TYPE_A_STAR=1

int8 FOOTSTEP_PLANNER_TYPE_VIS_GRAPH_WITH_A_STAR=2

int32 NO_PLAN_ID=-1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Starting left foot pose
geometry_msgs/Pose start_left_foot_pose

# Starting right foot pose
geometry_msgs/Pose start_right_foot_pose

# Goal left foot pose
geometry_msgs/Pose goal_left_foot_pose

# Goal right foot pose
geometry_msgs/Pose goal_right_foot_pose

# Requested initial stance side. If not specified the planner will choose
# Field default value 255
int8 requested_initial_stance_side

# If true, the planner will snap the provided goal steps. Otherwise the provided poses will be trusted as valid footholds.
# Field default value True
bool snap_goal_steps

# If snap_goal_steps is true and the goal steps can't be snapped, this specifies whether to abort or go ahead and plan.
bool abort_if_goal_step_snapping_fails

# Footstep planner type, see above
# Field default value 255
int8 requested_footstep_planner_type

# Acceptable xy distance from the given goal for the planner to terminate
# Field default value -1.0
float64 goal_distance_proximity

# Acceptable yaw offset from the given goal for the planner to terminate
# Field default value -1.0
float64 goal_yaw_proximity

# Planner timeout in seconds. If max_iterations is set also, the planner terminates whenever either is reached
# Field default value 5.0
float64 timeout

# Maximum iterations. Set to a non-positive number to disable. If timeout is also set, the planner terminates whener either is reached.
# Field default value -1
int32 max_iterations

# Best effort timeout in seconds
float64 best_effort_timeout

# Max body path length if using body path
float64 horizon_length

# Planar regions to use, if you don't want to assume flat ground
controller_msgs/PlanarRegionsListMessage planar_regions_list_message

# Explicitly tell the planner to use flat ground
bool assume_flat_ground

# Set this id to keep track of your request
# Field default value -1
int32 planner_request_id

# Requested body path waypoints. If non-empty, planner will follow this path and will not plan a body path
geometry_msgs/Pose[] body_path_waypoints

# Generate log of this plan. Logs are written to ~/.ihmc/logs by default, set the environment variable IHMC_FOOTSTEP_PLANNER_LOG_DIR to override this directory.
# For example, export IHMC_FOOTSTEP_PLANNER_LOG_DIR=/home/user/myLogs/
bool generate_log


