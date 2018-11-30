# This message is part of the IHMC whole-body controller API.
# The controller sends this message to notify that it has shifted the remaining footsteps to be executed due to some execution error.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The amount by which the remaining footsteps have been translated.
geometry_msgs/Vector3 offset_vector