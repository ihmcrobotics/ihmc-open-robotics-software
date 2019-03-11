# This message is part of the IHMC whole-body controller API.
# The controller sends this message to notify the user of the status of walking.

# The robot has begun its initial transfer/sway at the start of a walking plan.
byte STARTED = 0
# The robot has finished its final transfer/sway at the end of a walking plan.
byte COMPLETED = 1
# A walking abort has been requested by the controller.
byte ABORT_REQUESTED = 2
# The robot is back to standing on a break waiting for either an un-pause command or new footsteps.
byte PAUSED = 3
# The robot is resuming the series of footsteps that were paused.
byte RESUMED = 4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Status of walking.
byte walking_status 255