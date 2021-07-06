# This message is part of the IHMC multi-contact controller API.

int8 TRAJECTORY_STATUS_STARTED=0

int8 TRAJECTORY_STATUS_COMPLETED=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Reports the status of the trajectory
# Field default value 255
int8 trajectory_status


