## FootTrajectoryRosMessage
# This message commands the controller first to unload if necessary and then to move in taskspace a
# foot to the desired pose (position & orientation) while going through the specified trajectory
# points. A third order polynomial function is used to interpolate positions and a hermite based curve
# (third order) is used to interpolate the orientations. To excute a single straight line trajectory
# to reach a desired foot pose, set only one trajectory point with zero velocity and its time to be
# equal to the desired trajectory time. A message with a unique id equals to 0 will be interpreted as
# invalid and will not be processed by the controller. This rule does not apply to the fields of this
# message.

# Specifies which foot will execute the trajectory.
int8 robot_side

# The position/orientation trajectory information.
ihmc_msgs/SE3TrajectoryRosMessage se3_trajectory

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


