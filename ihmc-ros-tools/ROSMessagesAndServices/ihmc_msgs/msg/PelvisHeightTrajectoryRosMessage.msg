## PelvisHeightTrajectoryRosMessage
# This mesage commands the controller to move the pelvis to a new height in the trajectory frame while
# going through the specified trajectory points. Sending this command will not affect the pelvis
# horizontal position. To control the pelvis 3D position use the PelvisTrajectoryMessage instead. A
# message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the
# controller. This rule does not apply to the fields of this message.

# The position trajectory information.
ihmc_msgs/EuclideanTrajectoryRosMessage euclidean_trajectory

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


