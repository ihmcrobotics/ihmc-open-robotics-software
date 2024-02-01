# This message is part of the IHMC multi-contact controller API.
# Message for commanding whole-body jointspace trajectory as well as precomputed load status

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Timed intervals that left arm is in contact
ihmc_common_msgs/TimeIntervalMessage[] left_arm_contact_intervals

# Timed intervals that right arm is in contact
ihmc_common_msgs/TimeIntervalMessage[] right_arm_contact_intervals

# Timed intervals that left leg is in contact
ihmc_common_msgs/TimeIntervalMessage[] left_leg_contact_intervals

# Timed intervals that right leg is in contact
ihmc_common_msgs/TimeIntervalMessage[] right_leg_contact_intervals


