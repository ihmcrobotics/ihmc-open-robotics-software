# General purpose message that contains vertices to represent a 2D polygon (z ignored).

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

geometry_msgs/Point[<=100] vertices