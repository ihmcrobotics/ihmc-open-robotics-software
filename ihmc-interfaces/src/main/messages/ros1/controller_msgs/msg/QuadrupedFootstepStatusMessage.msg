int8 FOOTSTEP_STATUS_STARTED=0

int8 FOOTSTEP_STATUS_COMPLETED=1

int8 FRONT_LEFT=0

int8 FRONT_RIGHT=1

int8 HIND_RIGHT=2

int8 HIND_LEFT=3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The current footstep status enum value.
# Field default value 255
int8 footstep_status

# footstep_index starts at 0 and monotonically increases during a controller session
int32 footstep_index

# The robot quadrant that this footstep status correlates to.
# Field default value 255
int8 robot_quadrant

# Specifies the desired position of the foot sent to the controller as opposed to where the foot actually landed.
geometry_msgs/Point desired_touchdown_position_in_world

# Specifies the position of where the foot actually landed, only valid when the status is FOOTSTEP_STATUS_COMPLETED
geometry_msgs/Point actual_touchdown_position_in_world

# Specifies the desired timing of the step
controller_msgs/TimeIntervalMessage desired_step_interval

# Specifies the achieved timing of the step, the end time is only valid when the status is FOOTSTEP_STATUS_COMPLETED
controller_msgs/TimeIntervalMessage actual_step_interval


