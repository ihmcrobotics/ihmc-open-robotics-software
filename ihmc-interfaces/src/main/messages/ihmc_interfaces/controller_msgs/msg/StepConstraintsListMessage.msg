# This message is part of the IHMC robot environment awareness module.
# This message contains a list of constraint regions.
# A constraint region is a finite area that lies on a 3D plane and that is delimited by a concave hull.
# As concave hulls are complex to manipulate, a set of convex polygons are also provided, altogether they approximate the area of the constraint region.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The coordinates in world of the origin of each constraint region.
geometry_msgs/Point[<=20] region_origin
# The orientation with respect to world of each constraint region.
geometry_msgs/Quaternion[<=20] region_orientation
# The normal expressed in world of each constraint region.
geometry_msgs/Vector3[<=20] region_normal

# Indicates for each constraint region the number of vertices for the concave hull.
int32[<=100] concave_hulls_size
# Indicates for each constraint region the number of holes contained in that region.
int32[<=100] number_of_holes_in_region
# Indicates successively the number of vertices for each hole of each constraint region.
int32[<=100] hole_polygons_size

# Single buffer used to store for each constraint the vertices of the concave hull and the convex polygons.
geometry_msgs/Point[<=50000] vertex_buffer