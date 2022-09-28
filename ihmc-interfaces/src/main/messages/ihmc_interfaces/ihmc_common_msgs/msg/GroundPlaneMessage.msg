# This message is part of the IHMC quadruped controller API.
# This message contains the controller's estimate of the ground plane.
# It is defined by an origin and a normal

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

geometry_msgs/Point region_origin
geometry_msgs/Vector3 region_normal