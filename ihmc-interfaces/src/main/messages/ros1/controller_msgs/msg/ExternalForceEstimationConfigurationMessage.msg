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

# The is the unique hash code of the end-effector that is assumed to be pushed
# See RigidBody.hashCode() for the computation of a rigid-body hash code.
int32 end_effector_hash_code

# This is the position externally applied force to be estimated, in endEffector.getBodyFixedFrame().
geometry_msgs/Point external_force_position


