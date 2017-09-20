## SO3TrajectoryPointRosMessage
# This class is used to build trajectory messages in taskspace. It holds the only the rotational
# information for one trajectory point (orientation & angular velocity). Feel free to look at
# EuclideanTrajectoryPointMessage (translational) and SE3TrajectoryPointMessage (rotational AND
# translational)

# Time at which the trajectory point has to be reached. The time is relative to when the trajectory
# starts.
float64 time

# Define the desired 3D orientation to be reached at this trajectory point.
geometry_msgs/Quaternion orientation

# Define the desired 3D angular velocity to be reached at this trajectory point.
geometry_msgs/Vector3 angular_velocity

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


