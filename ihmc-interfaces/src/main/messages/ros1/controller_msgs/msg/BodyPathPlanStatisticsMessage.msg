int32 NO_PLAN_ID=-1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value -1
int32 plan_id

controller_msgs/VisibilityMapWithNavigableRegionMessage[] navigable_regions

controller_msgs/VisibilityMapMessage inter_regions_map

controller_msgs/VisibilityMapMessage start_visibility_map

controller_msgs/VisibilityMapMessage goal_visibility_map


