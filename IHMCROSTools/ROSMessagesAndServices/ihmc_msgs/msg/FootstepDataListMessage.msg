## FootstepDataListMessage
# This message commands the controller to execute a list of footsteps. See FootstepDataMessage
# for information about defining a footstep.

FootstepDataMessage[] footstep_data_list

# swingTime is the time spent in single-support when stepping
float64 swing_time

# transferTime is the time spent in double-support between steps
float64 transfer_time

# A unique id for the current message. This can be a timestamp or sequence number.
# Only the unique id in the top level message is used, the unique id in nested messages is ignored.
# Use /output/last_received_message for feedback about when the last message was received.
int64 unique_id


