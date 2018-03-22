# This message is part of the IHMC robot environment awareness module.
# This message contains a single planar region.
# It is defined by an origin, a normal, and a concave hull.
# For easier processing, a set of convex polygons approximating the concave hull are also provided.

int32 NO_REGION_ID = -1

int32 region_id -1
geometry_msgs/Point region_origin
geometry_msgs/Vector3 region_normal
Polygon2DMessage concave_hull
Polygon2DMessage[] convex_polygons 