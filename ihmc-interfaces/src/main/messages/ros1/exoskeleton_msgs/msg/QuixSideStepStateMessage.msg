# This message is part of the IHMC whole-body controller API.
# This message is used to notify the crutch display of the current side-step walking state.

int8 STANDING=0

int8 TO_SWING_OUT_LEFT_SUPPORT=1

int8 TO_SWING_OUT_RIGHT_SUPPORT=2

int8 SWING_OUT_LEFT_SUPPORT=3

int8 SWING_OUT_RIGHT_SUPPORT=4

int8 TO_SWING_CLOSE_LEFT_SUPPORT=5

int8 TO_SWING_CLOSE_RIGHT_SUPPORT=6

int8 SWING_CLOSE_LEFT_SUPPORT=7

int8 SWING_CLOSE_RIGHT_SUPPORT=8

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies which state the controller should transition into.
# Field default value 255
int8 side_step_state_name


