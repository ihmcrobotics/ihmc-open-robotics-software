# This message is part of the IHMC whole-body controller API.
# This message gives the status of the current footstep from the controller as well as the position and orientation of the footstep in world coordinates.

byte FOOTSTEP_STATUS_STARTED = 0
byte FOOTSTEP_STATUS_COMPLETED = 1

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The current footstep status enum value.
byte footstep_status 255
# footstep_index starts at 0 and monotonically increases with each completed footstep in a given FootstepDataListMessage.
int32 footstep_index
# The robot side (left or right) that this footstep status correlates to.
byte robot_side 255
# Specifies the desired position of the foot sent to the controller as opposed to where the foot actually landed.
geometry_msgs/Point desired_foot_position_in_world
# Specifies the desired orientation of the foot sent to the controller as opposed to where the foot actually landed.
geometry_msgs/Quaternion desired_foot_orientation_in_world
# Specifies the position of where the foot actually landed.
geometry_msgs/Point actual_foot_position_in_world
# Specifies the orientation of where the foot actually landed.
geometry_msgs/Quaternion actual_foot_orientation_in_world
# This is the swing duration of the step.
float64 swing_duration