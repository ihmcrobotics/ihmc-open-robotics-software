# This message is part of the IHMC external force estimation module.
# It provides the result from the estimator

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Estimated external force in world frame
geometry_msgs/Vector3 estimated_external_force

# An indicator of the quality of the estimation
# Field default value -1.0
float64 solution_quality


