## FootstepStatusRosMessage
# This message gives the status of the current footstep from the controller as well as the position
# and orientation of the footstep in world cooredinates. 

# The current footstep status enum value.
uint8 status

# footstepIndex starts at 0 and monotonically increases with each completed footstep in a given
# FootstepDataListMessage.
int32 footstep_index

# The robot side (left or right) that this footstep status correlates to.
uint8 robot_side

# desiredFootPositionInWorld gives the position of the desired position sent to the controller as
# opposed to where the foot actually landed
geometry_msgs/Vector3 desired_foot_position_in_world

# desiredFootOrientationInWorld gives the desired orientation of the foot sent to the controller as
# opposed to the orientation where the foot actually is
geometry_msgs/Quaternion desired_foot_orientation_in_world

# actualFootPositionInWorld gives the position of where the foot actually landed as opposed to the
# desired position sent to the controller
geometry_msgs/Vector3 actual_foot_position_in_world

# actualFootOrientationInWorld gives the orientation the foot is actually in as opposed to the desired
# orientation sent to the controller
geometry_msgs/Quaternion actual_foot_orientation_in_world

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "status" enum values:
uint8 STARTED=0 # execution of a footstep has begun. actualFootPositionInWorld and actualFootOrientationInWorld should be ignored in this state
uint8 COMPLETED=1 # a footstep is completed

# "robot_side" enum values:
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot

