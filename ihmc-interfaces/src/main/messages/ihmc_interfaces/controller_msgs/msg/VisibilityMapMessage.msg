# This message is part of the IHMC footstep planning module.

# unique id of the map
int32 map_id

# start point for the connection
geometry_msgs/Point[] source_points
int32[] source_region_ids

# target point for the connection
geometry_msgs/Point[] target_points
int32[] target_region_ids
