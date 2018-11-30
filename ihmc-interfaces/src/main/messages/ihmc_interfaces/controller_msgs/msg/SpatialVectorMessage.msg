# Represents a 6D vector composed of a 3D anuglar vector and a 3D linear vector.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

geometry_msgs/Vector3 angular_part
geometry_msgs/Vector3 linear_part