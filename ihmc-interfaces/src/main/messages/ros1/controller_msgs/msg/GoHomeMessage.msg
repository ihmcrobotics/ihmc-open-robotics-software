int8 HUMANOID_BODY_PART_ARM=0

int8 HUMANOID_BODY_PART_CHEST=1

int8 HUMANOID_BODY_PART_PELVIS=2

int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the part of the body the use wants to move back to its home configuration.
# Field default value 255
int8 humanoid_body_part

# Needed to identify a side dependent end-effector.
# Field default value 255
int8 robot_side

# How long the trajectory will spline from the current desired to the home configuration.
float64 trajectory_time

# The time to delay this command on the controller side before being executed.
float64 execution_delay_time


