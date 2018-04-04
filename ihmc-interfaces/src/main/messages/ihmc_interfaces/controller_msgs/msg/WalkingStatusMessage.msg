# This message is part of the IHMC whole-body controller API.
# The controller sends this message to notify the user of the status of walking.

byte STARTED = 0
byte COMPLETED = 1
byte ABORT_REQUESTED = 2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Status of walking.
byte walking_status 255