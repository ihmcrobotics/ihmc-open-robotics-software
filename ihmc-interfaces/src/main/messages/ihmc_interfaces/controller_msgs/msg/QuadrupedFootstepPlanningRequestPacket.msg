# This message is part of the IHMC quadruped footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int32 NO_PLAN_ID = -1

byte FOOTSTEP_PLANNER_TYPE_SIMPLE_PATH_TURN_WALK_TURN = 0
byte FOOTSTEP_PLANNER_TYPE_VIS_GRAPH_WITH_TURN_WALK_TURN = 1
byte FOOTSTEP_PLANNER_TYPE_A_STAR = 2


geometry_msgs/Point body_position_in_world
geometry_msgs/Quaternion body_orientation_in_world
geometry_msgs/Point goal_position_in_world
geometry_msgs/Quaternion goal_orientation_in_world
PlanarRegionsListMessage planar_regions_list_message
int32 planner_request_id -1
byte requested_footstep_planner_type 255