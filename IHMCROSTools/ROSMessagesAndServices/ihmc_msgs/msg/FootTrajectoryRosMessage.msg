## FootTrajectoryRosMessage
# This message commands the controller first to unload if necessary and then to move in taskspace a
# foot to the desired pose (position & orientation) while going through the specified trajectory
# points. A third order polynomial function is used to interpolate positions and a hermite based curve
# (third order) is used to interpolate the orientations. To excute a single straight line trajectory
# to reach a desired foot pose, set only one trajectory point with zero velocity and its time to be
# equal to the desired trajectory time. A message with a unique id equals to 0 will be interpreted as
# invalid and will not be processed by the controller. This rule does not apply to the fields of this
# message.

# Specifies the which foot will execute the trajectory.
uint8 robot_side

# List of trajectory points (in taskpsace) to go through while executing the trajectory. All the
# information contained in these trajectory points needs to be expressed in world frame.
ihmc_msgs/SE3TrajectoryPointRosMessage[] taskspace_trajectory_points

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

