# This message is part of the IHMC whole-body controller API.
# This message commands the controller to execute a list of footsteps.
# See FootstepDataMessage for more information about defining a footstep.

int8 STEP_TYPE_FLAT_WALKING=0

int8 STEP_TYPE_STAIRS=1

int8 STEP_TYPE_STEPPING_STONES=2

int8 STEP_TYPE_SLOPES=3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Defines the list of footstep to perform.
exoskeleton_msgs/ExoStepDataMessage[] step_data_list

# The swing_duration is the time a foot is not in ground contact during a step.
# Each step in a list of footsteps might have a different swing duration.
# The value specified here is a default value, used if a footstep in this list was created without a swing_duration.
# When set to zero or a negative value, the controller will its own default value.
# Field default value -1.0
float64 default_swing_duration

# The transfer_duration is the time spent with the feet in ground contact before a step.
# Each step in a list of footsteps might have a different transfer duration.
# The value specified here is a default value, used if a footstep in this list was created without a transfer-duration.
# When set to zero or a negative value, the controller will its own default value.
# Field default value -1.0
float64 default_transfer_duration

# Properties for queueing footstep lists.
ihmc_common_msgs/QueueableMessage queueing_properties

int8 step_type


