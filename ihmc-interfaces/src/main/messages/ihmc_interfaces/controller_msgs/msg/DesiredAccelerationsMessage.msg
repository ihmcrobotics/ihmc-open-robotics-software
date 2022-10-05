# This message is part of the IHMC whole-body controller API.
# General message that carries desired joint accelerations.
# It is used by ArmDesiredAccelerationsMessage, SpineDesiredAccelerationsMessage, NeckDesiredAccelerationsMessage.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the desired joint accelerations.
float64[] desired_joint_accelerations
# Properties for queueing commands.
ihmc_common_msgs/QueueableMessage queueing_properties
