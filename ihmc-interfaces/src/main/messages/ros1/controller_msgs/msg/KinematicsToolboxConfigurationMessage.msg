# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Indicates whether the privileged_root_joint_position is to be used or not.
bool use_privileged_root_joint_position

# Indicates whether the privileged_root_joint_orientation is to be used or not.
bool use_privileged_root_joint_orientation

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
# A value of -1 will result in the solver using its default value.
# Field default value -1.0
float64 privileged_weight

# The feedback proportional gain to use for the privileged configuration.
# It is coupled to some extent to the privileged_weight
# A value of -1 will result in the solver using its default value.
# Field default value -1.0
float64 privileged_gain

# Specifies how much high joint velocity values should be penalized in the optimization problem.
# A low value generally results in a reduce number of iterations before convergence but it also decreases the general stability of the solver.
# A value of -1 will result in the solver using its default value.
# Field default value -1.0
float64 joint_velocity_weight

# Specifying how much high joint acceleration values should be penalized in the optimization problem.
# A value of -1 will result in the solver using its default value.
# Field default value -1.0
float64 joint_acceleration_weight


