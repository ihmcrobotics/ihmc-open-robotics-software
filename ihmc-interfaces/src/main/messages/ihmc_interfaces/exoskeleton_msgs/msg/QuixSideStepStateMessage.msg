# This message is part of the IHMC whole-body controller API.
# This message is used to notify the crutch display of the current side-step walking state.

byte STANDING = 0
byte TO_SWING_OUT_LEFT_SUPPORT = 1
byte TO_SWING_OUT_RIGHT_SUPPORT = 2
byte SWING_OUT_LEFT_SUPPORT = 3
byte SWING_OUT_RIGHT_SUPPORT = 4
byte TO_SWING_CLOSE_LEFT_SUPPORT = 5
byte TO_SWING_CLOSE_RIGHT_SUPPORT = 6
byte SWING_CLOSE_LEFT_SUPPORT = 7
byte SWING_CLOSE_RIGHT_SUPPORT = 8


# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies which state the controller should transition into.
byte side_step_state_name 255