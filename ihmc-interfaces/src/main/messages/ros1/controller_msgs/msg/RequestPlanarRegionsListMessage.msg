int8 SINGLE_UPDATE=0

int8 CONTINUOUS_UPDATE=1

int8 STOP_UPDATE=2

int8 CLEAR=3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int8 planar_regions_request_type

controller_msgs/BoundingBox3DMessage bounding_box_in_world_for_request


