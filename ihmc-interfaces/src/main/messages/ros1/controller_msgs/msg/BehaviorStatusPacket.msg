# This message is part of the IHMC humanoid behavior module.

int8 NO_BEHAVIOR_RUNNING=0

int8 BEHAVIOR_RUNNING=1

int8 BEHAVIOR_PAUSED=2

int8 BEHAVIOR_FINISHED_FAILED=3

int8 BEHAVIOR_FINISHED_SUCCESS=4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 255
int8 current_behavior_status

# Field default value 255
int8 humanoid_behavior_type


