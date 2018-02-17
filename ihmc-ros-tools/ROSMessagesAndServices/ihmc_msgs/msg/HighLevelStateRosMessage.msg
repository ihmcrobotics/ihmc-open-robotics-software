## HighLevelStateRosMessage
# This message is used to switch the control scheme between force and position control. WARNING: When
# in position control, the IHMC balance algorithms will be disabled and it is up to the user to ensure
# stability.

# The enum value of the current high level state of the robot.
int8 high_level_controller_name

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


