# This message is part of the IHMC robot environment awareness module.
# This message contains a list of planar regions.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int32[<=300] region_id
geometry_msgs/Point[<=300] region_origin
geometry_msgs/Vector3[<=300] region_normal
int32[<=300] concave_hulls_size
int32[<=300] number_of_convex_polygons
int32[<=1000] convex_polygons_size

geometry_msgs/Point[<=5000] vertex_buffer