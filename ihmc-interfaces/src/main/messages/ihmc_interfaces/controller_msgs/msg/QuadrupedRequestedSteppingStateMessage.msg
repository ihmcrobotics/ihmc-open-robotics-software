# This message is part of the IHMC quadruped controller API.
# This message is used to switch the control scheme between different stepping modes.

byte REQUEST_STAND = 0
byte REQUEST_STEP = 1
byte REQUEST_SOLE_WAYPOINT = 2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the which state the controller should transition into.
byte quadruped_stepping_state 255
