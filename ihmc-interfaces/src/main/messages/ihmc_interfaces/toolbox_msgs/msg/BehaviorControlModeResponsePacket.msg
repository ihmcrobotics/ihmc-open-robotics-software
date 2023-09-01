# This message is part of the IHMC humanoid behavior module.

byte STOP = 0
byte PAUSE = 1
byte RESUME = 2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte behavior_control_mode_enum_request 255