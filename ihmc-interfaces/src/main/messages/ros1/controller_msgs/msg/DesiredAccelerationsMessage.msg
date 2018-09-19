# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the desired joint accelerations.
float64[] desired_joint_accelerations

# Properties for queueing commands.
controller_msgs/QueueableMessage queueing_properties


