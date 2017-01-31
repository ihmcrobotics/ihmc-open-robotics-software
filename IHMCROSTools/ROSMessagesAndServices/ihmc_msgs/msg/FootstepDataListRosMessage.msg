## FootstepDataListRosMessage
# This message commands the controller to execute a list of footsteps. See FootstepDataMessage for
# information about defining a footstep. A message with a unique id equals to 0 will be interpreted as
# invalid and will not be processed by the controller. This rule does not apply to the fields of this
# message.

# Defines the list of footstep to perform.
ihmc_msgs/FootstepDataRosMessage[] footstep_data_list

# When OVERRIDE is chosen:  - All previously sent footstep lists will be overriden, regardless of
# their execution mode  - A footstep can be overridden up to end of transfer. Once the swing foot
# leaves the ground it cannot be overridden  When QUEUE is chosen:  This footstep list will be queued
# with previously sent footstep lists, regardless of their execution mode  - The trajectory time and
# swing time of all queued footsteps will be overwritten with this message's values.
uint8 execution_mode

# swingTime is the time spent in single-support when stepping  - Queueing does not support varying
# swing times. If execution mode is QUEUE, all queued footsteps' swingTime will be overwritten  - The
# value can be overwritten by specifying a swing time in the FootstepDataMessage for each individual
# step.
float64 default_swing_time

# transferTime is the time spent in double-support between steps  - Queueing does not support varying
# transfer times. If execution mode is QUEUE, all queued footsteps' transferTime will be overwritten 
# - The value can be overwritten by specifying a transfer time in the FootstepDataMessage for each
# individual step.
float64 default_transfer_time

# Specifies the transfer time to go to standing after the execution of the footstep list is finished.
# If the value is negative the defaultTransfer time will be used.
float64 final_transfer_time

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "execution_mode" enum values:
uint8 OVERRIDE=0 # This message will override the previous.
uint8 QUEUE=1 # The previous message will first be executed before executing this message. When sending a series of queued messages, the very first has to be declared as OVERRIDE.

