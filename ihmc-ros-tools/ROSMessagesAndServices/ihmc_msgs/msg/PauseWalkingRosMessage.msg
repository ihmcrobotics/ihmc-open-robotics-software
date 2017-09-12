## PauseWalkingRosMessage
# This message pauses the execution of a list of footsteps. If this message is sent in the middle of
# executing a footstep, the robot will finish the step and pause when back in double support. A
# message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the
# controller.

# True to pause walking, false to unpause and resume an existing plan.
bool pause

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


