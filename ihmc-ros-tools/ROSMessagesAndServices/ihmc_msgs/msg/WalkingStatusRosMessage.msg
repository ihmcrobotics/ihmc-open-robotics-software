## WalkingStatusRosMessage
# This class is used to report the status of walking.

# Status of walking. Either STARTED, COMPLETED, or ABORT_REQUESTED.
uint8 status

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "status" enum values:
uint8 STARTED=0 # The robot has begun its initial transfer/sway at the start of a walking plan
uint8 COMPLETED=1 # The robot has finished its final transfer/sway at the end of a walking plan
uint8 ABORT_REQUESTED=2 # A walking abort has been requested

