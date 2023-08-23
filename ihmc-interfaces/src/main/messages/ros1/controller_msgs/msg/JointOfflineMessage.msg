# This message is part of the IHMC whole-body controller API.
# This message notifies the controller of a joint going offline, which causes the controller to go into an emergency fall-prevention mode.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Hash-code of the joints which are offline. Joints that are currently offline and not included in this list are re-enabled.
# The hash-code is computed from OneDoFJoint#hashcode().
int32[] joint_offline_hash_codes

# Nominal left foot contact points (in sole frame) given the set of offline joints
geometry_msgs/Point[] nominal_left_foot_contact_points_2d

# Nominal right foot contact points (in sole frame) given the set of offline joints
geometry_msgs/Point[] nominal_right_foot_contact_points_2d

# The time to delay this command on the controller side before being executed.
float64 execution_delay_time


