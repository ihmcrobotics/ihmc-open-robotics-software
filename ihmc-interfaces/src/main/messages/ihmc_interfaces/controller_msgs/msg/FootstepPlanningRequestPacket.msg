# This message is part of the IHMC footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

byte SWING_PLANNER_TYPE_NONE = 0
byte SWING_PLANNER_TYPE_POSITION = 1
byte SWING_PLANNER_TYPE_PROPORTION = 2

int32 NO_PLAN_ID = -1

# Starting left foot pose
geometry_msgs/Pose start_left_foot_pose
# Starting right foot pose
geometry_msgs/Pose start_right_foot_pose
# Starting left foothold. This field is optional and only should be populated when the initial stance foot has a partial foothold
geometry_msgs/Point[<=10] initial_left_contact_points_2d
# Starting right foothold. This field is optional and only should be populated when the initial stance foot has a partial foothold
geometry_msgs/Point[<=10] initial_right_contact_points_2d
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
# If plan_body_path is true and the planner fails, this specifies whether to abort or use a straight-line body path
bool abort_if_body_path_planner_fails

# If true, will plan a body path. If false, will follow a straight-line path to the goal
bool plan_body_path
# If true, does A* search. If false, a simple turn-walk-turn path is returned with no checks on step feasibility.
bool perform_a_star_search true
# Requested body path waypoints. If non-empty, planner will follow this path and will not plan a body path
geometry_msgs/Pose[<=50] body_path_waypoints

# (In beta) acceptable xy distance from the given goal for the planner to terminate
float64 goal_distance_proximity -1.0
# (In beta) acceptable yaw offset from the given goal for the planner to terminate
float64 goal_yaw_proximity -1.0
# Specifies the desired robot heading. Zero (default) is facing forward, pi is walking backwards, positive angles is facing left (right foot leads).
# The planner generates turn-walk-turn plans and this describes the robot's orientation during the walk portion.
float64 requested_path_heading 0.0
# Planner timeout in seconds. If max_iterations is set also, the planner terminates whenever either is reached
float64 timeout 5.0
# Maximum iterations. Set to a non-positive number to disable. If timeout is also set, the planner terminates whener either is reached.
int32 max_iterations -1
# Max body path length if using body path
float64 horizon_length
# Planar regions to use, if you don't want to assume flat ground
PlanarRegionsListMessage planar_regions_list_message
# Explicitly tell the planner to use flat ground
bool assume_flat_ground
# Set this id to keep track of your request
int32 planner_request_id -1
# Period of time in seconds the planner will publish it's status. If this is a non-positive number no status is published until it's completed.
float64 status_publish_period 1.0
# Specifies swing planner to use. See the above enumeration
byte requested_swing_planner 0

# Generate log of this plan. Logs are written to ~/.ihmc/logs by default, set the environment variable IHMC_FOOTSTEP_PLANNER_LOG_DIR to override this directory.
# For example, export IHMC_FOOTSTEP_PLANNER_LOG_DIR=/home/user/myLogs/
bool generate_log