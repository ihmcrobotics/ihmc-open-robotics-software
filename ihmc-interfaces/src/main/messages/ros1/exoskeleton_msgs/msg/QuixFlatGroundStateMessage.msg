# This message is part of the IHMC whole-body controller API.
# This message is used to notify the crutch display of the current flat-ground walking state.

int8 STANDING=0

int8 TO_WALKING_LEFT_SUPPORT=1

int8 TO_WALKING_RIGHT_SUPPORT=2

int8 TOE_OFF_TO_LEFT_SUPPORT=3

int8 TOE_OFF_TO_RIGHT_SUPPORT=4

int8 WALKING_LEFT_SUPPORT=5

int8 WALKING_RIGHT_SUPPORT=6

int8 TRANSFER_TO_STANDING=7

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies which state the controller should transition into.
# Field default value 255
int8 flat_ground_state_name


