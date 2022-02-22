# This message is part of the IHMC multi-contact controller API.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte TRAJECTORY_STATUS_STARTED = 0
byte TRAJECTORY_STATUS_COMPLETED = 1

# Reports the status of the trajectory
byte trajectory_status 255
