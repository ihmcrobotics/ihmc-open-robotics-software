# This message is part of the IHMC whole-body controller API.
# This message commands the quadruped planner to execute a path composed of the given sequence of waypoints.
# A waypoint represents a desired (x, y, yaw) position which the robot should pass through.
# A Hermite based curve (third order) is used to interpolate the waypoints.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# If true, the times are expressed in absolute time relative to the robot start
bool is_expressed_in_absolute_time true

# List of body path waypoint
EuclideanTrajectoryPointMessage[<=50] body_path_points