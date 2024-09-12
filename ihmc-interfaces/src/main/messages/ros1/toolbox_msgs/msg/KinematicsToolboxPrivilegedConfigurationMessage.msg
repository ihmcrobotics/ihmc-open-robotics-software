# This message is part of the IHMC whole-body inverse kinematics module.
# It contains auxiliary information that allows to customized the privileged configuration.
# The privileged configuration is a constant robot configuration that is used to complete the solver input when underconstrained.
# It also helps escaping singularities, for instance a bent configuration for the knee is an ideal privileged configuration that will help knee singularities.

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

# Specifies the value for the alpha parameter of the nullspace projection.
# The nullspace projection is computed using a damped least square solver which the alpha parameter is for.
# The alpha value controls the size of the singularity neighborhood that is used to project the privileged configuration.
# A small value will result in a small neighborhood (for the arms, the elbows will more easily go straight)
# and a large value will result in a large neighborhood (for the arms, the elbows will resist going straight).
# Any negative value will be ignored and result in the solver using its default value.
# Field default value -1.0
float64 nullspace_alpha

# The weight to use in the optimization for the privileged configuration.
# When remaining close to the privileged configuration is important, raise this weight to a value higher than the
# weight of the main objectives.
# Any negative value will be ignored and result in the solver using its default value.
# Field default value -1.0
float64 privileged_weight

# The feedback proportional gain to use for the privileged configuration.
# It is coupled to some extent to the privileged_weight
# Any negative value will be ignored and result in the solver using its default value.
# Field default value -1.0
float64 privileged_gain


