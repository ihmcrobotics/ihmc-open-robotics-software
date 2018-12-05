int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

int8 LOAD_BEARING_REQUEST_LOAD=0

int8 LOAD_BEARING_REQUEST_UNLOAD=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Needed to identify a side dependent end-effector.
# Field default value 255
int8 robot_side

# Whether the end-effector should be loaded or unloaded.
int8 load_bearing_request

# The time to delay this command on the controller side before being executed.
float64 execution_delay_time


