# This message is part of the IHMC whole-body controller API.
# This message notifies the controller of a joint going offline, which causes the controller to go into an emergency fall-prevention mode.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Hash-code of the joint which is offline. The hash-code is computed from OneDoFJoint#hashcode().
int32 joint_offline_hash_code