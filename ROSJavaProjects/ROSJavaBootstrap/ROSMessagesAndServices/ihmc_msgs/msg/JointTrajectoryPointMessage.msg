## JointTrajectoryPointMessage
# Sub-Packet for a point in an arm joint trajectory. It works similar to the
# trajectory_msgs/JointTrajectoryPoint message.

# Arm joint angles for this waypoint in order. For Atlas the controller assumes
# joint angles will be given in the following order: shoulder pitch, shoulder
# roll, elbow pitch, elbow roll, wrist pitch, wrist roll
float64[] positions

# Arm joint angular velocities for this waypoint in order.
float64[] velocities

# Time at which the waypoint is reached after the controller recieved the packet.
# A value of zero is not allowed.
float64 time


