# This message is part of the IHMC whole-body controller API.
# This message is used to notify the crutch display of the current slope state.

int8 STANDING=0

int8 TO_WALKING_LEFT_SUPPORT=1

int8 TO_WALKING_RIGHT_SUPPORT=2

int8 WALKING_LEFT_SUPPORT=3

int8 WALKING_RIGHT_SUPPORT=4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 255
int8 slope_state_name


