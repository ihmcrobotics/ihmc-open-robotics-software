## EuclideanTrajectoryPointRosMessage
# This class is used to build trajectory messages in taskspace. It holds the only the translational
# information for one trajectory point (position & linear velocity). Feel free to look at
# SO3TrajectoryPointMessage (rotational) and SE3TrajectoryPointMessage (rotational AND translational)

# Time at which the trajectory point has to be reached. The time is relative to when the trajectory
# starts.
float64 time

# Define the desired 3D position to be reached at this trajectory point. It is expressed in world
# frame.
geometry_msgs/Point position

# Define the desired 3D linear velocity to be reached at this trajectory point. It is expressed in
# world frame.
geometry_msgs/Vector3 linear_velocity

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


