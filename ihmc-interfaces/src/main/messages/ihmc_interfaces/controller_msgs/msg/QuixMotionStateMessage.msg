# This message is part of the IHMC whole-body controller API.
# This message is used to switch the control scheme between different control mode.

byte HOLD_POSITION = 0
byte SIT_DOWN = 1
byte STAND_UP = 2
byte MOVE_TO_START = 3
byte FLAT_GROUND_WALKING = 4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the which state the controller should transition into.
byte motion_state_name 255
