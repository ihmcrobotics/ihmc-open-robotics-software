# The robot has begun its initial transfer/sway at the start of a walking plan.
int8 STARTED=0

# The robot has finished its final transfer/sway at the end of a walking plan.
int8 COMPLETED=1

# A walking abort has been requested by the controller.
int8 ABORT_REQUESTED=2

# The robot is back to standing on a break waiting for either an un-pause command or new footsteps.
int8 PAUSED=3

# The robot is resuming the series of footsteps that were paused.
int8 RESUMED=4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Status of walking.
# Field default value 255
int8 walking_status


