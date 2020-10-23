# This message is part of the IHMC whole-body controller API.
# This message is used to notify the crutch display of the current slope state.

byte STANDING = 0
byte TO_WALKING_LEFT_SUPPORT = 1
byte TO_WALKING_RIGHT_SUPPORT = 2
byte WALKING_LEFT_SUPPORT = 3
byte WALKING_RIGHT_SUPPORT = 4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte slope_state_name 255
