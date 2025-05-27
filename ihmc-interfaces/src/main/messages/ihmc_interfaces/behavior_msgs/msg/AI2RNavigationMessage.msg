# SPATIAL RELATION TYPE
byte DEFAULT = 0
byte FRONT = 1
byte BEHIND = 2
byte LEFT = 3
byte RIGHT = 4

# The type of spatial relation with the secondary reference object as defined above
byte spatial_relation

# Secondary reference frame (object) used as the point of view that defines the spatial relation with the object
string pov_reference_frame_name

# Reference frame (object) to go to
string object_name

# The distance to the reference frame (located at the object's centroid)
float64 distance_to_object