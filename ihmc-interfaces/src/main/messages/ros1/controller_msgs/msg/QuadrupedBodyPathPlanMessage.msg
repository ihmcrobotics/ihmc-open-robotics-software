# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# If true, the times are expressed in absolute time relative to the robot start
# Field default value True
bool is_expressed_in_absolute_time

# List of body path waypoint
controller_msgs/EuclideanTrajectoryPointMessage[] body_path_points


