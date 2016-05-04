## ArmTrajectoryRosMessage
# This message commands the controller to move an arm in jointspace to the desired joint angles while
# going through the specified trajectory points. A third order polynomial function is used to
# interpolate between trajectory points. The jointTrajectoryMessages can have different waypoint times
# and different number of waypoints. If a joint trajectory message is empty, the controller will hold
# the last desired joint position while executing the other joint trajectories. A message with a
# unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
# This rule does not apply to the fields of this message.

# Specifies the side of the robot that will execute the trajectory.
uint8 robot_side

# List of trajectory points per joint. The expected joint ordering is from the closest joint to the
# chest to the closest joint to the hand.
ihmc_msgs/OneDoFJointTrajectoryRosMessage[] joint_trajectory_messages

# When OVERRIDE is chosen:  - The time of the first trajectory point can be zero, in which case the
# controller will start directly at the first trajectory point. Otherwise the controller will prepend
# a first trajectory point at the current desired position.  When QUEUE is chosen:  - The message must
# carry the ID of the message it should be queued to.  - The very first message of a list of queued
# messages has to be an OVERRIDE message.  - The trajectory point times are relative to the the last
# trajectory point time of the previous message.  - The controller will queue the joint trajectory
# messages as a per joint basis. The first trajectory point has to be greater than zero.
uint8 execution_mode

# Only needed when using QUEUE mode, it refers to the message Id to which this message should be
# queued to. It is used by the controller to ensure that no message has been lost on the way. If a
# message appears to be missing (previousMessageId different from the last message ID received by the
# controller), the motion is aborted. If previousMessageId == 0, the controller will not check for the
# ID of the last received message.
int64 previous_message_id

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "robot_side" enum values:
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot

# "execution_mode" enum values:
uint8 OVERRIDE=0 # This message will override the previous.
uint8 QUEUE=1 # The previous message will first be executed before executing this message. When sending a series of queued messages, the very first has to be declared as OVERRIDE.

