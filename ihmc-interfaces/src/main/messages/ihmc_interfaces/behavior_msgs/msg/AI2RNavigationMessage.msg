# SPATIAL RELATION TYPE
byte DEFAULT = 0
byte FRONT = 1
byte BEHIND = 2
byte LEFT = 3
byte RIGHT = 4

# The type of spatial relation with the secondary reference object as defined above
byte spatial_relation

# Goto action - Secondary reference object used to defined the spatial relation with reference_frame_name
string secondary_reference_frame_name

# Goto action - Reference frame for the action
string reference_frame_name

# Goto action - The distance to the frame
float64 distance_to_frame