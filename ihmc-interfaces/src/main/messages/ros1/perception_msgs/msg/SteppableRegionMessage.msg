# This message is part of the IHMC robot environment awareness module.
# This message contains a single planar region.
# It is defined by an origin, a normal, and a concave hull.
# For easier processing, a set of convex polygons approximating the concave hull are also provided.

int32 NO_REGION_ID=-1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Approximate last update time
ihmc_common_msgs/InstantMessage last_updated

# Field default value -1
int32 region_id

float64 foot_yaw

geometry_msgs/Point region_origin

geometry_msgs/Quaternion region_orientation

geometry_msgs/Vector3 region_normal

geometry_msgs/Point[] vertex_buffer

int32 concave_hull_size

perception_msgs/HeightMapMessage local_height_map


