# This is part of the IHMC Common message package.
# This class is used to build 1D trajectory messages including jointspace trajectory messages.
# For 3D trajectory points look at:
# - EuclideanTrajectoryMessage (translational),
# - SO3TrajectoryPointMessage (rotational),
# - SE3TrajectoryPointMessage (translational AND rotational).

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
float64 time
# Define the desired 1D position to be reached at this trajectory point.
float64 position
# Define the desired 1D velocity to be reached at this trajectory point.
float64 velocity
# Define the desired 1D acceleration to be reached at this trajectory point.
float64 acceleration