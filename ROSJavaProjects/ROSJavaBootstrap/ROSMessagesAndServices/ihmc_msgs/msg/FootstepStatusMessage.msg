## FootstepStatusMessage
# This message gives the status of the current footstep from the controller.

#Options for enum
# uint8 STARTED = 0
# uint8 COMPLETED = 1
uint8 status
# footstepIndex monotonically increases with each completed footstep in a given
# FootstepDataList and is then reset to 0 after all footsteps in the list are
# completed.
int32 footstepIndex

