# This message is part of the IHMC whole-body controller API.
# This message gives the status of the current footstep from the controller as well as the position and orientation of the footstep in world coordinates.

int8 FOOTSTEP_STATUS_STARTED=0

int8 FOOTSTEP_STATUS_COMPLETED=1

int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The current footstep status enum value.
# Field default value 255
int8 footstep_status

# footstep_index starts at 0 and monotonically increases with each completed footstep in a given FootstepDataListMessage.
int32 footstep_index

# The robot side (left or right) that this footstep status correlates to.
# Field default value 255
int8 robot_side

# Specifies the desired position of the foot sent to the controller as opposed to where the foot actually landed.
float64 desired_step_length

float64 desired_step_height

float64 desired_step_pitch


