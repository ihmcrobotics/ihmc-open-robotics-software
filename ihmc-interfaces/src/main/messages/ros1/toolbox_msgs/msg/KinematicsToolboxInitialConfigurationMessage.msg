# This message is part of the IHMC whole-body inverse kinematics module.
# It contains auxiliary information that allows to customize the initial configuration.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# This array is used to identify to which joint each angle in initial_joint_angles belongs to.
# See Joint.hashCode() for the computation of a joint hash code.
int32[] initial_joint_hash_codes

# When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
float32[] initial_joint_angles


