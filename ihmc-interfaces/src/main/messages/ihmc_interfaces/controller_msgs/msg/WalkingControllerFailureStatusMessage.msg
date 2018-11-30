# This message is part of the IHMC whole-body controller API.
# The controller will send this message when detecting a fall.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the estimated falling direction in 2D
geometry_msgs/Vector3 falling_direction