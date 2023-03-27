# This message is part of the IHMC environment modeling process.
# This message contains a list of steppable regions.
# It is defined by an origin, a normal, and a concave hull.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Approximate last update time
ihmc_common_msgs/InstantMessage last_updated

# foot yaw for the corresponding regions
float64[<=100] foot_yaw
# Indicates for each planar region the number of vertices for the concave hull.
int32[<=100] regions_per_yaw

# ID number referring to each steppable region, it should be unique.
int32[<=1000] region_id
# The coordinates in world of the origin of each steppable region
geometry_msgs/Point[<=1000] region_origin
# The orientation with respect to world of each steppable region.
geometry_msgs/Quaternion[<=1000] region_orientation
# The normal expressed in world of each steppable region.
geometry_msgs/Vector3[<=1000] region_normal

# Indicates for each planar region the number of vertices for the concave hull.
int32[<=100] concave_hulls_size
# Single buffer used to store for each planar the vertices of the concave hull and the convex polygons.
geometry_msgs/Point[<=3000] vertex_buffer

# Height map message used for each of the concave hulls
HeightMapMessage[<=100] local_height_map