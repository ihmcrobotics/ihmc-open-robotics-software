# This message is part of the IHMC whole-body controller API.
# The controller will send this message when detecting a fall.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

# Specifies the estimated falling direction in 2D
geometry_msgs/Vector3 falling_direction