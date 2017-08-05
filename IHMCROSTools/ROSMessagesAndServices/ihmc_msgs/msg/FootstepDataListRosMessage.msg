## FootstepDataListRosMessage
# This message commands the controller to execute a list of footsteps. See FootstepDataMessage for
# information about defining a footstep. A message with a unique id equals to 0 will be interpreted as
# invalid and will not be processed by the controller. This rule does not apply to the fields of this
# message.

# Defines the list of footstep to perform.
ihmc_msgs/FootstepDataRosMessage[] footstep_data_list

# When CONTROL_DURATIONS is chosen:  The controller will try to achieve the swingDuration and the
# transferDuration specified in the message. If a  footstep touches down early, the next step will not
# be affected by this and the whole trajectory might finish  earlier then expected. When
# CONTROL_ABSOLUTE_TIMINGS is chosen:  The controller will compute the expected times for swing start
# and touchdown and attempt to start a footstep  at that time. If a footstep touches down early, the
# following transfer will be extended to make up for this  time difference and the footstep plan will
# finish at the expected time.
uint8 execution_timing

# The swingDuration is the time a foot is not in ground contact during a step. Each step in a list of
# footsteps might have a different swing duration. The value specified here is a default value, used
# if a footstep in this list was created without a swingDuration.
float64 default_swing_duration

# The transferDuration is the time spent with the feet in ground contact before a step. Each step in a
# list of footsteps might have a different transfer duration. The value specified here is a default
# value, used if a footstep in this list was created without a transferDuration.
float64 default_transfer_duration

# Specifies the time used to return to a stable standing stance after the execution of the footstep
# list is finished. If the value is negative the defaultTransferDuration will be used.
float64 final_transfer_duration

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

# "execution_mode" enum values:
uint8 OVERRIDE=0 # This message will override the previous.
uint8 QUEUE=1 # The previous message will first be executed before executing this message. When sending a series of queued messages, the very first has to be declared as OVERRIDE.

# "execution_timing" enum values:
uint8 CONTROL_DURATIONS=0 # During the execution of this message the controller will attempt to achieve the given durations for segments of the whole trajectory.
uint8 CONTROL_ABSOLUTE_TIMINGS=1 # During the execution of this message the controller will attempt to achieve the absolute timings at the knot points relative to the start of execution.

