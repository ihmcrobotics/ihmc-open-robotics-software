
# SPATIAL RELATION TYPE
int8 DEFAULT=0

int8 FRONT=1

int8 BEHIND=2

int8 LEFT=3

int8 RIGHT=4

# The type of spatial relation with the secondary reference object as defined above
int8 spatial_relation

# Secondary reference frame (object) used as the point of view that defines the spatial relation with the object
string pov_reference_frame_name

# Reference frame (object) to go to
string object_name

# The distance to the reference frame (located at the object's centroid)
float64 distance_to_object


