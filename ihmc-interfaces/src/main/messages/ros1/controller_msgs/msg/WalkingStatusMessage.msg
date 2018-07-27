int8 STARTED=0

int8 COMPLETED=1

int8 ABORT_REQUESTED=2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Status of walking.
# Field default value 255
int8 walking_status


