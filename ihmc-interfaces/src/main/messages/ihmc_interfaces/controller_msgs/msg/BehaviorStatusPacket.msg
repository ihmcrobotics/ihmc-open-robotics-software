# This message is part of the IHMC humanoid behavior module.

byte NO_BEHAVIOR_RUNNING = 0
byte BEHAVIOR_RUNNING = 1
byte BEHAVIOR_PAUSED = 2
byte BEHAVIOR_FINISHED_FAILED = 3
byte BEHAVIOR_FINISHED_SUCCESS = 4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte current_behavior_status 255

# Field default value 255
int8 humanoid_behavior_type