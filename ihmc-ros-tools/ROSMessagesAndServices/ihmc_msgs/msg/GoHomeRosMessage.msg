## GoHomeRosMessage
# The message commands the controller to bring the given part of the body back to a default
# configuration called 'home'. It is useful to get back to a safe configuration before walking.

# Specifies the part of the body the user wants to move back to it home configuration.
int8 humanoid_body_part

# Needed to identify a side dependent end-effector.
int8 robot_side

# How long the trajectory will spline from the current desired to the home configuration.
float64 trajectory_time

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


