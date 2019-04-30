int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

int8 FOOTSTEP_PLANNER_TYPE_PLANAR_REGION_BIPEDAL=0

int8 FOOTSTEP_PLANNER_TYPE_PLAN_THEN_SNAP=1

int8 FOOTSTEP_PLANNER_TYPE_A_STAR=2

int8 FOOTSTEP_PLANNER_TYPE_SIMPLE_BODY_PATH=3

int8 FOOTSTEP_PLANNER_TYPE_VIS_GRAPH_WITH_A_STAR=4

int32 NO_PLAN_ID=-1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 255
int8 initial_stance_robot_side

geometry_msgs/Point stance_foot_position_in_world

geometry_msgs/Quaternion stance_foot_orientation_in_world

geometry_msgs/Point goal_position_in_world

geometry_msgs/Quaternion goal_orientation_in_world

# Field default value 255
int8 requested_footstep_planner_type

float64 timeout

float64 horizon_length

controller_msgs/PlanarRegionsListMessage planar_regions_list_message

bool assume_flat_ground

# Field default value -1
int32 planner_request_id

# Radius around the given goal pose that defines a goal region.
# A step is considered inside the goal region if it's associated mid-foot point
# is inside the region.
# If this value is non-positive the goal region will consist of a single point
# at the given pose
# Field default value -1.0
float64 goal_position_proximity


