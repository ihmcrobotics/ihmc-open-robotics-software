# This message is part of the IHMC hole-body inverse kinematics module.
# It contains auxiliary information that allows to further customized the behavior of the solver.

# When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
geometry_msgs/Point privileged_root_joint_position
# When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
geometry_msgs/Quaternion privileged_root_joint_orientation
# This array is used to identify to which joint each angle in privileged_joint_angles belongs to.
# See AbstractInverseDynamicsJoint.getNameBaseHashCode() for the computation of a joint hash code.
int64[] privileged_joint_name_based_hash_codes
# When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
float32 privileged_joint_angles