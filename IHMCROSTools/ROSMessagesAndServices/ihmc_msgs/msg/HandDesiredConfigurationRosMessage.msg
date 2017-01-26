## HandDesiredConfigurationRosMessage
# Packet for commanding the hands to perform various predefined grasps. A message with a unique id
# equals to 0 will be interpreted as invalid and will not be processed by the controller.

# Specifies the side of the robot that will execute the trajectory
uint8 robot_side

# Specifies the grasp to perform
uint8 hand_desired_configuration

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "robot_side" enum values:
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot

# "hand_configuration" enum values:
uint8 STOP=0 # stops the fingers at their current position
uint8 OPEN=1 # fully opens the fingers
uint8 CLOSE=2 # fully closes the fingers
uint8 CRUSH=3 # fully closes the fingers applying maximum force
uint8 HOOK=4 # closes all but one finger to create a hook
uint8 BASIC_GRIP=5 # sets gripper to use a standard grasp
uint8 PINCH_GRIP=6 # sets gripper to use a pinch grasp where the thumb and fingers come together when closed
uint8 WIDE_GRIP=7 # sets gripper to use a wide-spread finger grasp
uint8 SCISSOR_GRIP=8 # sets gripper to use a scissor grasp where the index and middle finger come together when closed
uint8 RESET=9 # sets all fingers to their zero position
uint8 OPEN_FINGERS=10 # fully open all fingers except the thumb
uint8 OPEN_THUMB=11 # fully open the thumb only
uint8 CLOSE_FINGERS=12 # fully close all fingers except the thumb
uint8 CLOSE_THUMB=13 # fully close the thumb only

