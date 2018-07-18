# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int32[] region_id

geometry_msgs/Point[] region_origin

geometry_msgs/Vector3[] region_normal

int32[] concave_hulls_size

int32[] number_of_convex_polygons

int32[] convex_polygons_size

geometry_msgs/Point[] vertex_buffer


