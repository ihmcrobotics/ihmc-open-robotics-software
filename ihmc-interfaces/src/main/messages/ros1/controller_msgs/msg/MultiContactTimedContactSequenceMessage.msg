# This message is part of the IHMC multi-contact controller API.
# Message for commanding whole-body jointspace trajectory as well as precomputed load status

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Timed contact sequences of left arm
ihmc_common_msgs/TimeIntervalMessage[] left_arm_contact_intervals

# Timed contact sequences of right arm
ihmc_common_msgs/TimeIntervalMessage[] right_arm_contact_intervals

# Timed contact sequences of left leg
ihmc_common_msgs/TimeIntervalMessage[] left_leg_contact_intervals

# Timed contact sequences of right leg
ihmc_common_msgs/TimeIntervalMessage[] right_leg_contact_intervals


