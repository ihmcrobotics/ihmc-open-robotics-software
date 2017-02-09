## GoHomeRosMessage
# The message commands the controller to bring the given part of the body back to a default
# configuration called 'home'. It is useful to get back to a safe configuration before walking.

# Specifies the part of the body the user wants to move back to it home configuration.
uint8 body_part

# Needed to identify a side dependent end-effector.
uint8 robot_side

# How long the trajectory will spline from the current desired to the home configuration.
float64 trajectory_time

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "robot_side" enum values:
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot

# "body_part" enum values:
uint8 ARM=0 # Request the chest to go back to a straight up configuration.
uint8 CHEST=1 # Request the arm to go to a preconfigured home configuration that is elbow lightly flexed, forearm pointing forward, and upper pointing downward.
uint8 PELVIS=2 # Request the pelvis to go back to between the feet, zero pitch and roll, and headed in the same direction as the feet.

