# This message is part of the IHMC toolbox framework.

byte WAKE_UP = 0
byte REINITIALIZE = 1
byte SLEEP = 2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte requested_toolbox_state 255