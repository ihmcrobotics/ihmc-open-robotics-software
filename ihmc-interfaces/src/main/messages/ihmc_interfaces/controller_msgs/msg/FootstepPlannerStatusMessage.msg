# This message is part of the IHMC footstep planner API.
# The footstep planner sends this message to notify the user of the status of planning.

byte IDLE = 0
byte PLANNING_PATH = 1
byte PLANNING_STEP= 2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Status of planning.
byte planning_status 255