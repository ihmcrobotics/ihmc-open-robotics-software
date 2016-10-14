## StopAllTrajectoryRosMessage
# Stop the execution of any trajectory being executed. A message with a unique id equals to 0 will be
# interpreted as invalid and will not be processed by the controller.

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


