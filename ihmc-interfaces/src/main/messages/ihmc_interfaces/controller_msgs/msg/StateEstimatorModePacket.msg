# This message is part of the IHMC whole-body controller API.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte NORMAL = 0
byte FROZEN = 1

byte requested_state_estimator_mode 255