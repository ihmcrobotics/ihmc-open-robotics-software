# This message is part of the IHMC whole-body controller API.
# Request the whole-body controller to track a desired trajectory for the center of mass.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# List of center of mass trajectory waypoints. Each waypoint contains the center of mass position and velocity at a given time.
ihmc_common_msgs/EuclideanTrajectoryMessage euclidean_trajectory


