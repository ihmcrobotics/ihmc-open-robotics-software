int32 NO_PLAN_ID=-1

int8 FOOTSTEP_PLANNER_TYPE_SIMPLE_PATH_TURN_WALK_TURN=0

int8 FOOTSTEP_PLANNER_TYPE_VIS_GRAPH_WITH_TURN_WALK_TURN=1

int8 FOOTSTEP_PLANNER_TYPE_A_STAR=2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

geometry_msgs/Point body_position_in_world

geometry_msgs/Quaternion body_orientation_in_world

geometry_msgs/Point goal_position_in_world

geometry_msgs/Quaternion goal_orientation_in_world

controller_msgs/PlanarRegionsListMessage planar_regions_list_message

# Field default value -1
int32 planner_request_id

# Field default value 255
int8 requested_footstep_planner_type


