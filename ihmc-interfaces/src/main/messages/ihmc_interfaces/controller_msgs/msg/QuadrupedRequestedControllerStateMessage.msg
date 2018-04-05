# This message is part of the IHMC quadruped controller API.
# This message is used to switch the control scheme between different control mode.

byte REQUEST_DO_NOTHING = 0
byte REQUEST_STAND_PREP = 1
byte REQUEST_FREEZE = 2
byte REQUEST_STEPPING = 3
byte REQUEST_FALL = 4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the which state the controller should transition into.
byte quadruped_controller_name 255
