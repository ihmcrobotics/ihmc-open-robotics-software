# This message is part of the Quix controller API.

byte SQUARE = 0
byte SHORT_FORWARD = 1
byte MEDIUM_FORWARD = 2
byte LONG_FORWARD = 3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte flat_step_type_name 255