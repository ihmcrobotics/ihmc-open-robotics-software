# This message is part of the IHMC whole-body controller API.
# This message is used to notify the crutch display of the current stand-up state.

int8 WAITING=0

int8 MOVE_TO_INITIAL_POSITION_FOR_STAND_UP=1

int8 WAIT_TO_STAND_UP=2

int8 STAND_UP=3

int8 DONE=4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 255
int8 stand_up_state_name


