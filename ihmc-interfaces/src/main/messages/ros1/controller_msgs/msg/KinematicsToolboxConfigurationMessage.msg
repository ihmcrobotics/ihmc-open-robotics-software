# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
geometry_msgs/Point privileged_root_joint_position

# When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
geometry_msgs/Quaternion privileged_root_joint_orientation

# This array is used to identify to which joint each angle in privileged_joint_angles belongs to.
# See AbstractInverseDynamicsJoint.getNameBaseHashCode() for the computation of a joint hash code.
int64[] privileged_joint_name_based_hash_codes

# When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
float32[] privileged_joint_angles


