# This message is part of the IHMC external force estimation module.
# It provides the result from the estimator

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Estimated external force in world frame
geometry_msgs/Vector3[] estimated_external_forces

# Estimated root joint wrench, if requested. Will be set to NaN if root joint was not included in the solver
geometry_msgs/Wrench estimated_root_joint_wrench

# Estimated contacting rigid body hashcode, if configured with estimate_contact_location=true
int32 rigid_body_hash_code

# Estimated contact point in parent joint frame, if configured with estimate_contact_location=true
geometry_msgs/Point contact_point


