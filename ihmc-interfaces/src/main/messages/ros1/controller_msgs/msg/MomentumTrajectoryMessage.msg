# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# List of angular momentum trajectory waypoints.
# Each waypoint contains the angular momentum and the angular momentum rate at a given time.
# NOTE: As the angular momentum can't be encoded using the SO3TrajectoryMessage, the EuclideanTrajectoryMessage is used.
controller_msgs/EuclideanTrajectoryMessage angular_momentum_trajectory


