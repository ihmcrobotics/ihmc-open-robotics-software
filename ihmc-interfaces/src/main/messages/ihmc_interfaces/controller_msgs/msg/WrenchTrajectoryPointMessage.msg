# This message is part of the IHMC whole-body controller API.
# This class is used to build a wrench (force & moment) profile over time.
# It holds the necessary information for one trajectory point.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
float64 time
# Define the desired wrench (force & moment) to be achieved at this trajectory point.
geometry_msgs/Wrench wrench