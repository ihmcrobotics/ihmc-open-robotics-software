# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# If true, the times are expressed in absolute time relative to the robot start
# Field default value True
bool is_expressed_in_absolute_time

# Indicates if the given trajectory should be considered an "absolute" orientation or an "offset" orientation
# Field default value True
bool is_an_offset_orientation

# The orientation trajectory information.
controller_msgs/SO3TrajectoryMessage so3_trajectory


