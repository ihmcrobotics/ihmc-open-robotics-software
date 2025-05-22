
# SPATIAL RELATION TYPE
int8 DEFAULT=0

int8 FRONT=1

int8 BEHIND=2

int8 LEFT=3

int8 RIGHT=4

# The type of spatial relation with the secondary reference object as defined above
int8 spatial_relation

# Goto action - Secondary reference object used to defined the spatial relation with reference_frame_name
string secondary_reference_frame_name

# Goto action - Reference frame for the action
string reference_frame_name

# Goto action - The distance to the frame
float64 distance_to_frame


