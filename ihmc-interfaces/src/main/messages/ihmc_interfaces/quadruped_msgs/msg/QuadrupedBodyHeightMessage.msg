# This message is part of the IHMC quadruped controller API.
# This message commands the controller to move in taskspace the body to the desired height while going through the specified trajectory points.
# Sending this command will not affect the pelvis horizontal position.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# If true, the times are expressed in absolute time relative to the robot start
bool is_expressed_in_absolute_time false
# If true, the body height is controlled, rather than the center of mass height
bool control_body_height false
# The orientation trajectory information.
ihmc_common_msgs/EuclideanTrajectoryMessage euclidean_trajectory