# Set this id to keep track of your request
int32 planner_request_id -1

# Left foot pose
geometry_msgs/Pose left_foot_pose
# Right foot pose
geometry_msgs/Pose right_foot_pose
# Goal poses
geometry_msgs/Pose[] goal_poses

# Parameters for the planner to use
ValkyrieFootstepPlannerParametersPacket parameters

# Acceptable xy distance from the given goal for the planner to terminate
float64 goal_distance_proximity -1.0
# Acceptable yaw offset from the given goal for the planner to terminate
float64 goal_yaw_proximity -1.0
# Timeout in seconds
float64 timeout

# Planar regions to use, if you don't want to assume flat ground
PlanarRegionsListMessage planar_regions_list_message
# Explicitly tell the planner to use flat ground
bool assume_flat_ground
# If true a body path is computed as part of the planning process. If true, only one goal pose is used
bool compute_body_path
