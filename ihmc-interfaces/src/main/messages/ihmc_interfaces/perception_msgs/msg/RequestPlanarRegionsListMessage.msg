# This message is obsolete and will be removed in the near future.
# This message is part of the IHMC robot environment awareness (REA) module.
# Use this message to request new planar regions
# @deprecated REA always publishes planar regions when running. To request REA to clear its internal state, see REAStateRequestMessage.

byte SINGLE_UPDATE = 0
byte CONTINUOUS_UPDATE = 1
byte STOP_UPDATE = 2
byte CLEAR = 3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte planar_regions_request_type

ihmc_common_msgs/BoundingBox3DMessage bounding_box_in_world_for_request