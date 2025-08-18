
# SPATIAL RELATION TYPE
int8 DEFAULT=0

int8 FRONT=1

int8 BEHIND=2

int8 LEFT=3

int8 RIGHT=4

# The type of spatial relation with the secondary reference object as defined above
int8 spatial_relation

# Point of view object. Secondary reference frame (object) used as the point of view that defines the spatial relation with the object
string pov_object

# Target reference frame (object) to go to
string target_object

# The distance to the reference frame (located at the object's centroid)
float64 distance_to_object


