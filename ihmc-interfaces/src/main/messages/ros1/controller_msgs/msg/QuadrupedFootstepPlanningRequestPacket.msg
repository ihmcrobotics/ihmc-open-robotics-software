int32 NO_PLAN_ID=-1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

geometry_msgs/Point body_position_in_world

geometry_msgs/Quaternion body_orientation_in_world

geometry_msgs/Point goal_position_in_world

geometry_msgs/Quaternion goal_orientation_in_world

controller_msgs/PlanarRegionsListMessage planar_regions_list_message

# Field default value -1
int32 planner_request_id


