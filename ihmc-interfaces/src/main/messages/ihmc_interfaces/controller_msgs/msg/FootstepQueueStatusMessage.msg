# This message is part of the IHMC whole-body controller API.
# This message gives the status of the footstep queue in the controller in world coordinates.
# See QueuedFootstepStatusMessage for more information about defining a footstep.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies whether the first step in the queue list is currently being taken in swing
bool is_first_step_in_swing false
# Specifies the time in the current support sequence of the current step. This could be in transfer, or in swing
float64 time_in_support_sequence

# Defines the list of footsteps contained in the queue.
QueuedFootstepStatusMessage[<=50] queued_footstep_list
