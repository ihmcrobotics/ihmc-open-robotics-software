## FootLoadBearingRosMessage
# This message commands the controller to start loading a foot that was unloaded to support the robot
# weight.  When the robot is performing a 'flamingo stance' (one foot in the air not actually walking)
# and the user wants the robot to switch back to double support. A message with a unique id equals to
# 0 will be interpreted as invalid and will not be processed by the controller.

# Needed to identify a side dependent end-effector.
uint8 robot_side

# Wether the end-effector should be loaded or unloaded.
uint8 request

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "load_bearing_request" enum values:
uint8 LOAD=0 # Request to load the given end-effector.
uint8 UNLOAD=1 # Request to unload the given end-effector.

# "robot_side" enum values:
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot

