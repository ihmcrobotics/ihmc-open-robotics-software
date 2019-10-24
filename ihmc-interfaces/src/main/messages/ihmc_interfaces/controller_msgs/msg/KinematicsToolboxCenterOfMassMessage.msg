# This message is part of the IHMC whole-body inverse kinematics module.
# It holds all the information needed for detailing the type of constraint to apply to the center of mass.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the desired center of mass position.
# The data is assumed to be expressed in world frame.
geometry_msgs/Point desired_position_in_world
# The selection matrix is used to determinate which degree of freedom of the center of mass
# should be controlled.
# The selection frame coming along with the given selection matrix is used to determine to what
# reference frame the selected axes are referring to. For instance, if only the hand height in
# world should be controlled on the linear z component of the selection matrix should be
# selected and the reference frame should world frame. When no reference frame is provided with
# the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed frame
# if not defined otherwise.
SelectionMatrix3DMessage selection_matrix
# Specifies the priority of controller the position along each axis independently.
# If no frame is provided, the weight matrix will be applied in the center of mass frame which is
# aligned with the world axes.
WeightMatrix3DMessage weights