# This message is part of the IHMC whole-body controller API.
# This class is used to build 1D trajectory messages including jointspace trajectory messages.
# For 3D trajectory points look at:
# - EuclideanTrajectoryMessage (translational),
# - SO3TrajectoryPointMessage (rotational),
# - SE3TrajectoryPointMessage (translational AND rotational).

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

# Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
float64 time
# Define the desired 1D position to be reached at this trajectory point.
float64 position
# Define the desired 1D velocity to be reached at this trajectory point.
float64 velocity