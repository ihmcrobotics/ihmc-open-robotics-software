## FootstepDataListMessage
# This message gives a list of footsteps for the controller to execute.
# The position and orientatino of the first footstep should be in pelvis
# frame, with each additions footstep in the reference frame of the preceding
# footstep.

FootstepDataMessage[] footstepDataList
#Options for enum
# uint8 DEFAULT = 0
# uint8 BY_BOX = 1
# uint8 STEP_ON_OR_OFF = 2
# uint8 NO_STEP = 3
# uint8 LOW_HEIGHT = 4
uint8 trajectoryWaypointGenerationMethod
# trajectoryBoxData describes the position, orientation and dimension of
# the box used to generate the trajectory waypoints for the BY_BOX method
SquareDataMessage trajectoryBoxData
# swingTime is the time spent in single-support when executing footsteps
float64 swingTime
# transferTime is the time spent in double-support when executing footsteps
float64 transferTime

