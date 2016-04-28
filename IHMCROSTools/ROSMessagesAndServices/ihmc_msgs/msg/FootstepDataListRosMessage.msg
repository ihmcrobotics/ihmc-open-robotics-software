## FootstepDataListRosMessage
# This message commands the controller to execute a list of footsteps. See FootstepDataMessage for
# information about defining a footstep. A message with a unique id equals to 0 will be interpreted as
# invalid and will not be processed by the controller. This rule does not apply to the fields of this
# message.

# Defines the list of footstep to perform.
ihmc_msgs/FootstepDataRosMessage[] footstep_data_list

# swingTime is the time spent in single-support when stepping
float64 swing_time

# transferTime is the time spent in double-support between steps
float64 transfer_time

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


