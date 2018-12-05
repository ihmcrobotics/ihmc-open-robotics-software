# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# If true, the times are expressed in absolute time relative to the robot start
bool is_expressed_in_absolute_time

# If true, the body height is controlled, rather than the center of mass height
bool control_body_height

# The orientation trajectory information.
controller_msgs/EuclideanTrajectoryMessage euclidean_trajectory


