## FootstepStatusMessage
# This message gives the status of the current footstep from the controller.

#Options for status
# uint8 STARTED = 0
# uint8 COMPLETED = 1
uint8 status

# footstepIndex starts at 0 and monotonically increases with each completed footstep in a given
# FootstepDataListMessage.
int32 footstepIndex

int32 robotSide

geometry_msgs/Vector3 actualFootPositionInWorld

geometry_msgs/Quaternion actualFootOrientationInWorld

bool isDoneWalking

int8 destination


