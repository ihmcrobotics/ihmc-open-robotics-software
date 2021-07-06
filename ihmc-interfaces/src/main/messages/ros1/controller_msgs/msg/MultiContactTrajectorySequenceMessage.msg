# This message is part of the IHMC multi-contact controller API.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Sequence of trajectories to be executed in succession
controller_msgs/MultiContactTrajectoryMessage[] trajectory_sequence


