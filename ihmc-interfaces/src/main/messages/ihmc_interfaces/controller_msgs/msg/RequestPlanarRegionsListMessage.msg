# This message is part of the IHMC robot environment awareness module.
# Use this message to request new planar regions

byte SINGLE_UPDATE = 0
byte CONTINUOUS_UPDATE = 1
byte STOP_UPDATE = 2
byte CLEAR = 3

byte planar_regions_request_type

BoundingBox3DMessage bounding_box_in_world_for_request