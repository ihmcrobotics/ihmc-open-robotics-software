# This message is part of the IHMC whole-body controller API.
# This class is used to change the PID gains over time.
# It holds the necessary information for one trajectory point.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
float64 time
# Define the desired linear PID gains for the trajectory point.
PID3DGains linear_gains
# Define the desired angular PID gains for the trajectory point.
PID3DGains angular_gains