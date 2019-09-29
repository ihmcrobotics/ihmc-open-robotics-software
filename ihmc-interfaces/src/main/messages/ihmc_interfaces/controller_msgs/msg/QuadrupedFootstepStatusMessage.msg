# This message is part of the IHMC whole-body controller API.
# This message gives the status of the current footstep from the quadruped controller as well as the position of the footstep in world coordinates.

byte FOOTSTEP_STATUS_STARTED = 0
byte FOOTSTEP_STATUS_COMPLETED = 1

byte FRONT_LEFT = 0
byte FRONT_RIGHT = 1
byte HIND_RIGHT = 2
byte HIND_LEFT = 3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The current footstep status enum value.
byte footstep_status 255
# footstep_index starts at 0 and monotonically increases during a controller session
int32 footstep_index
# The robot quadrant that this footstep status correlates to.
byte robot_quadrant 255

# Specifies the desired position of the foot sent to the controller as opposed to where the foot actually landed.
geometry_msgs/Point desired_touchdown_position_in_world
# Specifies the position of where the foot actually landed, only valid when the status is FOOTSTEP_STATUS_COMPLETED
geometry_msgs/Point actual_touchdown_position_in_world
# Specifies the desired timing of the step
TimeIntervalMessage desired_step_interval
# Specifies the achieved timing of the step, the end time is only valid when the status is FOOTSTEP_STATUS_COMPLETED
TimeIntervalMessage actual_step_interval