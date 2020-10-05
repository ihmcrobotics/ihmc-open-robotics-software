# This message is part of the IHMC quadruped controller API.
# This message is used to switch the control scheme between different stepping modes.

int8 REQUEST_STAND=0

int8 REQUEST_STEP=1

int8 REQUEST_SOLE_WAYPOINT=2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the which state the controller should transition into.
# Field default value 255
int8 quadruped_stepping_requested_event


