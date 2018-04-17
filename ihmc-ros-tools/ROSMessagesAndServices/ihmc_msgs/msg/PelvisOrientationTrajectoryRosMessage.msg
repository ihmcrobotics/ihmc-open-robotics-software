## PelvisOrientationTrajectoryRosMessage
# This message commands the controller to move in taskspace the pelvis to the desired orientation
# while going through the specified trajectory points. A hermite based curve (third order) is used to
# interpolate the orientations. This message allows controlling the pelvis orientation without
# interferring with position that will still be controlled to maintain the current desired capture
# poit position. To excute a normal trajectory to reach a desired pelvis orientation, set only one
# trajectory point with zero velocity and its time to be equal to the desired trajectory time. A
# message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the
# controller. This rule does not apply to the fields of this message.

# Whether the pelvis orientation is allowed to controlled by the user when the robot is walking.
bool enable_user_pelvis_control_during_walking

# The orientation trajectory information.
ihmc_msgs/SO3TrajectoryRosMessage so3_trajectory

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


