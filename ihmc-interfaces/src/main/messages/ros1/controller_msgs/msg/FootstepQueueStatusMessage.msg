# This message is part of the IHMC whole-body controller API.
# This message gives the status of the footstep queue in the controller in world coordinates.
# See QueuedFootstepStatusMessage for more information about defining a footstep.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Defines the list of footsteps contained in the queue.
controller_msgs/QueuedFootstepStatusMessage[] queued_footstep_list


