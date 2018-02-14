## PelvisTrajectoryRosMessage
# This message commands the controller to move in taskspace the pelvis to the desired pose (position &
# orientation) while going through the specified trajectory points. A third order polynomial function
# is used to interpolate positions and a hermite based curve (third order) is used to interpolate the
# orientations. To excute a single straight line trajectory to reach a desired pelvis pose, set only
# one trajectory point with zero velocity and its time to be equal to the desired trajectory time.
# Note that the pelvis position is limited keep the robot's balance (center of mass has to remain
# inside the support polygon). A message with a unique id equals to 0 will be interpreted as invalid
# and will not be processed by the controller. This rule does not apply to the fields of this message.

# The position/orientation trajectory information.
ihmc_msgs/SE3TrajectoryRosMessage se3_trajectory

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


