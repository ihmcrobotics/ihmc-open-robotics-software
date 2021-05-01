# This message is part of the IHMC footstep planning module.
# The planner recomputes swing waypoints for the last planned path if it receives this message.
# If no path was previously planned, it's ignored.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte SWING_PLANNER_TYPE_NONE = 0
byte SWING_PLANNER_TYPE_POSITION = 1
byte SWING_PLANNER_TYPE_PROPORTION = 2

# Specifies swing planner to use. See the above enumeration
byte requested_swing_planner 0

# Generate log of this plan. Logs are written to ~/.ihmc/logs by default, set the environment variable IHMC_FOOTSTEP_PLANNER_LOG_DIR to override this directory.
# For example, export IHMC_FOOTSTEP_PLANNER_LOG_DIR=/home/user/myLogs/
bool generate_log