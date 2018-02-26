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

# If false the controller adjust each footstep height to be at the support sole height.
bool trust_height_of_footsteps

# Contains information on whether the robot can automatically adjust its footsteps to retain balance.
bool are_footsteps_adjustable

# If true the controller will adjust upcoming footsteps with the location error of previous steps.
bool offset_footsteps_with_execution_error

# Properties for queueing footstep lists.
ihmc_msgs/QueueableRosMessage queueing_properties

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "execution_timing" enum values:
uint8 CONTROL_DURATIONS=0 # During the execution of this message the controller will attempt to achieve the given durations for segments of the whole trajectory.
uint8 CONTROL_ABSOLUTE_TIMINGS=1 # During the execution of this message the controller will attempt to achieve the absolute timings at the knot points relative to the start of execution.

