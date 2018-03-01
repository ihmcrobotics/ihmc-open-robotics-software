# This message is part of the IHMC whole-body controller API.
# General message that carries desired joint accelerations.
# It is used by ArmDesiredAccelerationsMessage, SpineDesiredAccelerationsMessage, NeckDesiredAccelerationsMessage.

# Specifies the desired joint accelerations.
float64[] desired_joint_accelerations
# Properties for queueing commands.
QueueableMessage queueing_properties
