## FootstepDataListMessage
# This message commands the controller to execute a list of footsteps. See FootstepDataMessage
# for information about defining a footstep.

FootstepDataMessage[] footstepDataList

# DEFAULT gives the standard IHMC parabolic footstep trajectory
# BY_BOX gives a square-wave type trajectory according to the parameters of trajectoryBoxData
# STEP_ON_OR_OFF gives a higher-arched trajectory meant for stepping onto or off obstacles
# NO_STEP results in a null footstep. This probably should not be used outside of IHMC source
# LOW_HEIGHT gives a lower-arched trajectory
#Options for trajectoryWaypointGenerationMethod
# uint8 DEFAULT = 0
# uint8 BY_BOX = 1
# uint8 STEP_ON_OR_OFF = 2
# uint8 NO_STEP = 3
# uint8 LOW_HEIGHT = 4
uint8 trajectoryWaypointGenerationMethod

# trajectoryBoxData describes the position, orientation and dimension of
# the box used to generate the trajectory waypoints for the BY_BOX method
SquareDataMessage trajectoryBoxData

# swingTime is the time spent in single-support when stepping
float64 swingTime

# transferTime is the time spent in double-support between steps
float64 transferTime


