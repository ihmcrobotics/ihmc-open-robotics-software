# This message is part of the IHMC whole-body controller API.
# This message is used to notify the crutch display of the current sit-down state.

byte WAITING = 0
byte SIT_DOWN = 1
byte WAITING_FOR_SEATED = 2
byte DONE = 3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id


byte sit_down_state_name 255
