## HighLevelStateChangeStatusRosMessage
# This message notifies the user of a change in the high level state. This message's primary use is to
# signal a requested state change is completed.

# initialState gives the controller's state prior to transition
int8 initial_high_level_controller_name

# endState gives the state the controller has transitioned into
int8 end_high_level_controller_name

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


