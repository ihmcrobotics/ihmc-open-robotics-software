# This message is part of the IHMC hole-body inverse kinematics module.
# It contains auxiliary information that allows to further customized the behavior of the solver.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
geometry_msgs/Point privileged_root_joint_position
# When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
geometry_msgs/Quaternion privileged_root_joint_orientation
# This array is used to identify to which joint each angle in privileged_joint_angles belongs to.
# See Joint.hashCode() for the computation of a joint hash code.
int32[] privileged_joint_hash_codes
# When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
float32[] privileged_joint_angles
# The weight to use in the optimization for the privileged configuration.
# When remaining close to the privileged configuration is important, raise this weight to a value higher than the 
# weight of the main objectives.
# Any value less than zero will be ignored.
float64 privileged_weight -1
# The feedback proportional gain to use for the privileged configuration.
# It is coupled to some extent to the privileged_weight
float64 privileged_gain -1