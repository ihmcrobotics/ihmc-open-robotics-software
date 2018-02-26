## AdjustFootstepRosMessage
# The intent of this message is to adjust a footstep when the robot is executing it (a foot is
# currently swinging to reach the footstep to be adjusted).

# Specifies which foot is expected to be executing the footstep to be adjusted.
uint8 robot_side

# Specifies the adjusted position of the footstep. It is expressed in world frame.
geometry_msgs/Point location

# Specifies the adjusted orientation of the footstep. It is expressed in world frame.
geometry_msgs/Quaternion orientation

# predictedContactPoints specifies the vertices of the expected contact polygon between the foot and
# the world. A value of null or an empty list will default to keep the contact points used for the
# original footstep. Contact points  are expressed in sole frame. This ordering does not matter. For
# example: to tell the controller to use the entire foot, the predicted contact points would be:
# predicted_contact_points: - {x: 0.5 * foot_length, y: -0.5 * toe_width} - {x: 0.5 * foot_length, y:
# 0.5 * toe_width} - {x: -0.5 * foot_length, y: -0.5 * heel_width} - {x: -0.5 * foot_length, y: 0.5 *
# heel_width} 
ihmc_msgs/Point2dRosMessage[] predicted_contact_points

# The time to delay this command on the controller side before being executed.
float64 execution_delay_time

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "robot_side" enum values:
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot

