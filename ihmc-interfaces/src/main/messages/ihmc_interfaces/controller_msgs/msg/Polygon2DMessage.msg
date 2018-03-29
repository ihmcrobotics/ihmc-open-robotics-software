# General purpose message that contains vertices to represent a 2D polygon (z ignored).

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

geometry_msgs/Point[<=100] vertices