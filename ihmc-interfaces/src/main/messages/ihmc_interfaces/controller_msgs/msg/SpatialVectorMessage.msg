# Represents a 6D vector composed of a 3D anuglar vector and a 3D linear vector.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

geometry_msgs/Vector3 angular_part
geometry_msgs/Vector3 linear_part