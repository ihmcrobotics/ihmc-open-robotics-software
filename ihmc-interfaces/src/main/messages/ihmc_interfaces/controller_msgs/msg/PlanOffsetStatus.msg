# This message is part of the IHMC whole-body controller API.
# The controller sends this message to notify that it has shifted the remaining footsteps to be executed due to some execution error.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

# The amount by which the remaining footsteps have been translated.
geometry_msgs/Vector3 offset_vector