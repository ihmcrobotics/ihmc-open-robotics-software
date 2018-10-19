# This message is part of the IHMC footstep planner API.
# The footstep planner sends this message to notify the user of the status of planning.

byte FOOTSTEP_PLANNER_STATUS_IDLE = 0
byte FOOTSTEP_PLANNER_STATUS_PLANNING_PATH = 1
byte FOOTSTEP_PLANNER_STATUS_PLANNING_STEPS = 2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte footstep_planner_status 255