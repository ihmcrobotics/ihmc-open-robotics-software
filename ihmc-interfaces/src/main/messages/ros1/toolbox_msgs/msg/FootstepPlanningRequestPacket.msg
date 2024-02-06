# This message is part of the IHMC footstep planning module.

int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

int8 SWING_PLANNER_TYPE_NONE=0

int8 SWING_PLANNER_TYPE_MULTI_WAYPOINT_POSITION=1

int8 SWING_PLANNER_TYPE_PROPORTION=2

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

# If plan_body_path is true and the planner fails, this specifies whether to abort or use a straight-line body path
bool abort_if_body_path_planner_fails

# If true, will plan a body path. If false, will follow a straight-line path to the goal
bool plan_body_path

# If true, will plan footsteps. If false, will not plan footsteps
# Field default value True
bool plan_footsteps

# If true, does A* search. If false, a simple turn-walk-turn path is returned with no checks on step feasibility.
# Field default value True
bool perform_a_star_search

# Requested body path waypoints in world-frame. If non-empty, planner will follow this path and will not plan a body path
# The robot will walk with the orientation of waypoint N between points N and N+1
geometry_msgs/Pose[] body_path_waypoints

# (In beta) acceptable xy distance from the given goal for the planner to terminate
# Field default value -1.0
float64 goal_distance_proximity

# (In beta) acceptable yaw offset from the given goal for the planner to terminate
# Field default value -1.0
float64 goal_yaw_proximity

# Planner timeout in seconds. If max_iterations is set also, the planner terminates whenever either is reached
# Field default value 5.0
float64 timeout

# Maximum iterations. Set to a non-positive number to disable. If timeout is also set, the planner terminates whener either is reached.
# Field default value -1
int32 max_iterations

# Max body path length if using body path
float64 horizon_length

# Height map data, if you don'ot want to assume flat ground.
perception_msgs/HeightMapMessage height_map_message

# If true, steps are snapped assuming flat ground.
# Note that collision checks will still be performed if enabled, such as FootstepPlannerParametersPacket.checkForBodyBoxCollisions
bool assume_flat_ground

# Set this id to keep track of your request
# Field default value -1
int32 planner_request_id

# Period of time in seconds the planner will publish it's status. If this is a non-positive number no status is published until it's completed.
# Field default value 1.0
float64 status_publish_period

# Specifies swing planner to use. See the above enumeration
int8 requested_swing_planner

# Reference plan to be used in ReferenceBasedIdealStepCalculator.
controller_msgs/FootstepDataListMessage reference_plan

# Generate log of this plan. Logs are written to ~/.ihmc/logs by default, set the environment variable IHMC_FOOTSTEP_PLANNER_LOG_DIR to override this directory.
# For example, export IHMC_FOOTSTEP_PLANNER_LOG_DIR=/home/user/myLogs/
bool generate_log


