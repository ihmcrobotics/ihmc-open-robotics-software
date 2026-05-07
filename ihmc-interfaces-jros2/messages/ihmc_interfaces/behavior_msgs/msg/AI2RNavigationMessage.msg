# SPATIAL RELATION TYPE
byte DEFAULT = 0
byte FRONT = 1
byte BEHIND = 2
byte LEFT = 3
byte RIGHT = 4

# The type of spatial relation with the secondary reference object as defined above
byte spatial_relation

# Point of view object. Secondary reference frame (object) used as the point of view that defines the spatial relation with the object
string pov_object

# Target reference frame (object) to go to
string target_object

# The distance to the reference frame (located at the object's centroid)
float64 distance_to_object

# The minimum distance to the obstacles considered in body path planning
float64 obstacle_clearance