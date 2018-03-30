# This message is part of the IHMC humanoid behavior module.

byte NO_BEHAVIOR_RUNNING = 0
byte BEHAVIOS_RUNNING = 1
byte BEHAVIOR_PAUSED = 2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte current_behavior_status 255