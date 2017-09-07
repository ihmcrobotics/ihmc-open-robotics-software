## TrajectoryPoint1DRosMessage
# This class is used to build 1D trajectory messages including jointspace trajectory messages. For 3D
# trajectory points look at EuclideanTrajectoryMessage (translational), SO3TrajectoryPointMessage
# (rotational), and SE3TrajectoryPointMessage (translational AND rotational).

# Time at which the trajectory point has to be reached. The time is relative to when the trajectory
# starts.
float64 time

# Define the desired 1D position to be reached at this trajectory point.
float64 position

# Define the desired 1D velocity to be reached at this trajectory point.
float64 velocity

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


