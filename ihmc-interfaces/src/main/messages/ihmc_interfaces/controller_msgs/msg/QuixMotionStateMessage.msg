# This message is part of the Quix controller API.
# This message is used to notify the crutch display of the current motion state and allow the crutch to communicate a desired change in state.

byte HOLD_POSITION = 0
byte SIT_DOWN = 1
byte STAND_UP = 2
byte MOVE_TO_FLAT_GROUND = 3
byte FLAT_GROUND_WALKING = 4
byte MOVE_TO_SLOPE = 5
byte SLOPE_WALKING = 6
byte OLD_FLAT_GROUND_WALKING = 7

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte motion_state_name 255
