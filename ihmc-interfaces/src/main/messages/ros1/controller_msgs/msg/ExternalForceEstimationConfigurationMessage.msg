# This message is part of the IHMC external force estimation module.
# It specifies where an expected external force will be applied.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Estimator gain, usually in the range of 0.25 - 5.0. If the system is noisy or prone to oscillation, a lower gain is suggested.
# Field default value 0.5
float64 estimator_gain

# Specifies the alpha value used by the damped least-squares solver, usually in the range (1e-6 - 1e-2). For long joint paths, smaller alphas are recommended.
# Field default value 0.005
float64 solver_alpha

# Indicates whether the wrench at the root joint should be included in the solver
# Field default value True
bool calculate_root_joint_wrench

# List of unique hash codes corresponding to the rigid bodies at which the solver will calculate external forces.
# See RigidBody.hashCode() for calculation of the hash code
int32[] rigid_body_hash_codes

# List of contact positions for each rigid body, expressed in RigidBody.getParentJoint().getFrameAfterJoint()
geometry_msgs/Point[] contact_point_positions

# (in beta) Estimates the location of the contact point at a single rigid body (the first provided). The root joint wrench is not estimated.
bool estimate_contact_location


