# This message is part of the IHMC whole-body controller API.
# This message is used to notify the crutch display of the current sit-down state.

int8 WAITING=0

int8 SIT_DOWN=1

int8 WAITING_FOR_SEATED=2

int8 DONE=3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 255
int8 sit_down_state_name


