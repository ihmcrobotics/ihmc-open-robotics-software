# This message is part of the IHMC robot environment awareness module.
# This message contains a list of planar regions.
# A planar region is a finite area that lies on a 3D plane and that is delimited by a concave hull.
# As concave hulls are complex to manipulate, a set of convex polygons are also provided, altogether they approximate the area of the planar region.  

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# ID number referring to each planar region, it should be unique.
int32[<=3000] region_id
# The coordinates in world of the origin of each planar region.
geometry_msgs/Point[<=3000] region_origin
# The orientation with respect to world of each planar region.
geometry_msgs/Quaternion[<=3000] region_orientation
# The normal expressed in world of each planar region.
geometry_msgs/Vector3[<=3000] region_normal

# Indicates for each planar region the number of vertices for the concave hull.
int32[<=3000] concave_hulls_size
# Indicates for each planar region the number of convex polygons.
int32[<=3000] number_of_convex_polygons
# Indicates successively the number of vertices for each convex polygon of each planar region.
int32[<=3000] convex_polygons_size

# Single buffer used to store for each planar the vertices of the concave hull and the convex polygons.
geometry_msgs/Point[<=50000] vertex_buffer