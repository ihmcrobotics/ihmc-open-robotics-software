# This message is part of the IHMC footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int32 NO_PLAN_ID = -1

int32 plan_id -1

VisibilityMapWithNavigableRegionMessage[<=25] navigable_regions

VisibilityMapMessage inter_regions_map
VisibilityMapMessage start_visibility_map
VisibilityMapMessage goal_visibility_map