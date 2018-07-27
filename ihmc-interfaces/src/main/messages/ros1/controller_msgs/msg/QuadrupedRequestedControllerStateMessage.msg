int8 REQUEST_DO_NOTHING=0

int8 REQUEST_STAND_PREP=1

int8 REQUEST_FREEZE=2

int8 REQUEST_STEPPING=3

int8 REQUEST_FALL=4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the which state the controller should transition into.
# Field default value 255
int8 quadruped_controller_requested_event


