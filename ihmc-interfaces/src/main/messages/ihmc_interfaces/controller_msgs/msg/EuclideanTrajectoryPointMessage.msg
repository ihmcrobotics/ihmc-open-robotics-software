# This message is part of the IHMC whole-body controller API.
# This message is used to build trajectory messages in taskspace.
# It holds the only the translational information for one trajectory point (position & linear velocity).
# Feel free to look at SO3TrajectoryPointMessage (rotational) and SE3TrajectoryPointMessage (rotational AND translational).

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
float64 time
# Define the desired 3D position to be reached at this trajectory point. It is expressed in world frame.
geometry_msgs/Point position
# Define the desired 3D linear velocity to be reached at this trajectory point. It is expressed in world frame.
geometry_msgs/Vector3 linear_velocity