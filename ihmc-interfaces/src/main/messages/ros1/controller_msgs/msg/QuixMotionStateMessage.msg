# This message is part of the IHMC whole-body controller API.
# This message is used to switch the control scheme between different control mode.

int8 HOLD_POSITION=0

int8 SIT_DOWN=1

int8 STAND_UP=2

int8 MOVE_TO_START=3

int8 FLAT_GROUND_WALKING=4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the which state the controller should transition into.
# Field default value 255
int8 motion_state_name


