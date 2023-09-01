# This message is part of the IHMC whole-body controller API.
# This message is used to notify the crutch display of the current flat-ground walking state.

byte STANDING = 0
byte TO_WALKING_LEFT_SUPPORT = 1
byte TO_WALKING_RIGHT_SUPPORT = 2
byte TOE_OFF_TO_LEFT_SUPPORT = 3
byte TOE_OFF_TO_RIGHT_SUPPORT = 4
byte WALKING_LEFT_SUPPORT = 5
byte WALKING_RIGHT_SUPPORT = 6
byte TRANSFER_TO_STANDING = 7

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies which state the controller should transition into.
byte flat_ground_state_name 255
