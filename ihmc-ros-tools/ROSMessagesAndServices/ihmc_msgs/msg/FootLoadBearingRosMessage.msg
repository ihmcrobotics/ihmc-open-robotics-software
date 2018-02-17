## FootLoadBearingRosMessage
# This message commands the controller to start loading a foot that was unloaded to support the robot
# weight.  When the robot is performing a 'flamingo stance' (one foot in the air not actually walking)
# and the user wants the robot to switch back to double support. A message with a unique id equals to
# 0 will be interpreted as invalid and will not be processed by the controller.

# Needed to identify a side dependent end-effector.
int8 robot_side

# Wether the end-effector should be loaded or unloaded.
int8 load_bearing_request

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


