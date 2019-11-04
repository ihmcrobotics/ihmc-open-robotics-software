int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

int8 FOOTSTEP_PLANNER_TYPE_PLANAR_REGION_BIPEDAL=0

int8 FOOTSTEP_PLANNER_TYPE_PLAN_THEN_SNAP=1

# The recommended planner type
int8 FOOTSTEP_PLANNER_TYPE_A_STAR=2

int8 FOOTSTEP_PLANNER_TYPE_SIMPLE_BODY_PATH=3

int8 FOOTSTEP_PLANNER_TYPE_VIS_GRAPH_WITH_A_STAR=4

int32 NO_PLAN_ID=-1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Initial stance foot side
# Field default value 255
int8 initial_stance_robot_side

# Stance foot sole frame z up position, z must be correct
geometry_msgs/Point stance_foot_position_in_world

# Stance foot sole frame z up orientation, only the "yaw" is considered
geometry_msgs/Quaternion stance_foot_orientation_in_world

# Goal mid feet z up position
geometry_msgs/Point goal_position_in_world

# Goal mid feet z up orientation, only the "yaw" is considered
geometry_msgs/Quaternion goal_orientation_in_world

# Footstep planner type, see above
# Field default value 255
int8 requested_footstep_planner_type

# Acceptable xy distance from the given goal for the planner to terminate
# Field default value -1.0
float64 goal_distance_proximity

# Acceptable yaw offset from the given goal for the planner to terminate
# Field default value -1.0
float64 goal_yaw_proximity

# Timeout in seconds
float64 timeout

# Max body path length if using body path
float64 horizon_length

# Planar regions to use, if you don't want to assume flat ground
controller_msgs/PlanarRegionsListMessage planar_regions_list_message

# Explicitly tell the planner to use flat ground
bool assume_flat_ground

# Set this id to keep track of your request
# Field default value -1
int32 planner_request_id


