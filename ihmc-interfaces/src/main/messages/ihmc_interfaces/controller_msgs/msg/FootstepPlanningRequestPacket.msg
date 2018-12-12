# This message is part of the IHMC footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

byte FOOTSTEP_PLANNER_TYPE_PLANAR_REGION_BIPEDAL = 0
byte FOOTSTEP_PLANNER_TYPE_PLAN_THEN_SNAP = 1
byte FOOTSTEP_PLANNER_TYPE_A_STAR = 2
byte FOOTSTEP_PLANNER_TYPE_SIMPLE_BODY_PATH = 3
byte FOOTSTEP_PLANNER_TYPE_VIS_GRAPH_WITH_A_STAR = 4

int32 NO_PLAN_ID = -1

byte initial_stance_robot_side 255
geometry_msgs/Point stance_foot_position_in_world
geometry_msgs/Quaternion stance_foot_orientation_in_world
geometry_msgs/Point goal_position_in_world
geometry_msgs/Quaternion goal_orientation_in_world
byte requested_footstep_planner_type 255
float64 timeout
float64 horizon_length
PlanarRegionsListMessage planar_regions_list_message
int32 planner_request_id -1