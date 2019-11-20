# This message is part of the IHMC localization module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

geometry_msgs/Pose pose
geometry_msgs/Vector3 linear_velocity
geometry_msgs/Vector3 angular_velocity

int64 timestamp
float64 confidence_factor
string frame_id