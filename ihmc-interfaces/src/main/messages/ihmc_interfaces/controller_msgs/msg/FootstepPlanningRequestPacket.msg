# This message is part of the IHMC footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

byte FOOTSTEP_PLANNER_TYPE_PLANAR_REGION_BIPEDAL = 0
byte FOOTSTEP_PLANNER_TYPE_PLAN_THEN_SNAP = 1
# The recommended planner type
byte FOOTSTEP_PLANNER_TYPE_A_STAR = 2
byte FOOTSTEP_PLANNER_TYPE_SIMPLE_BODY_PATH = 3
byte FOOTSTEP_PLANNER_TYPE_VIS_GRAPH_WITH_A_STAR = 4

int32 NO_PLAN_ID = -1

# Initial stance foot side
byte initial_stance_robot_side 255
# Stance foot sole frame z up position, z must be correct
geometry_msgs/Point stance_foot_position_in_world
# Stance foot sole frame z up orientation, only the "yaw" is considered
geometry_msgs/Quaternion stance_foot_orientation_in_world
# Goal mid feet z up position
geometry_msgs/Point goal_position_in_world
# Goal mid feet z up orientation, only the "yaw" is considered
geometry_msgs/Quaternion goal_orientation_in_world
# Footstep planner type, see above
byte requested_footstep_planner_type 255
# Acceptable xy distance from the given goal for the planner to terminate
float64 goal_distance_proximity -1.0
# Acceptable yaw offset from the given goal for the planner to terminate
float64 goal_yaw_proximity -1.0
# Timeout in seconds
float64 timeout
# Max body path length if using body path
float64 horizon_length
# Planar regions to use, if you don't want to assume flat ground
PlanarRegionsListMessage planar_regions_list_message
# Explicitly tell the planner to use flat ground
bool assume_flat_ground
# Set this id to keep track of your request
int32 planner_request_id -1