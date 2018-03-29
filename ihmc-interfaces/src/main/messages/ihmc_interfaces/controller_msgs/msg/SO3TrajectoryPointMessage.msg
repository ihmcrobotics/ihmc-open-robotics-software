# This message is part of the IHMC whole-body controller API.
# This class is used to build trajectory messages in taskspace.
# It holds the necessary information for one trajectory point.
# Feel free to look at EuclideanTrajectoryPointMessage (translational) and SE3TrajectoryPointMessage (rotational AND translational).

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
float64 time
# Define the desired 3D orientation to be reached at this trajectory point.
geometry_msgs/Quaternion orientation
# Define the desired 3D angular velocity to be reached at this trajectory point.
geometry_msgs/Vector3 angular_velocity