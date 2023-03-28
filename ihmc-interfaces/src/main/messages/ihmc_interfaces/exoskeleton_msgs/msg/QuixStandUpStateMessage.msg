# This message is part of the IHMC whole-body controller API.
# This message is used to notify the crutch display of the current stand-up state.

byte WAITING = 0
byte MOVE_TO_INITIAL_POSITION_FOR_STAND_UP = 1
byte WAIT_TO_STAND_UP = 2
byte STAND_UP = 3
byte DONE = 4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte stand_up_state_name 255
