# This message is part of the IHMC whole-body inverse kinematics module.
# This message contains all the information needed to configure the objectives/constraints to apply on 
# a given end-effector in the solver.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The is the unique hash code of the end-effector to be solved for.
# It is used on the solver side to retrieve the desired end-effector to be controlled.
# See RigidBody.hashCode() for the computation of a rigid-body hash code.
int32 end_effector_hash_code
# This is the desired position of the control frame's origin.
# If the control frame has not been defined, it represents the desired position endEffector.getBodyFixedFrame().
# The data is assumed to be expressed in world frame.
geometry_msgs/Point desired_position_in_world
# This is the desired orientation of the control frame.
# I the control frame has not been defined, it represents the desired orientation of endEffector.getBodyFixedFrame().
# The data is assumed to be expressed in world frame.
geometry_msgs/Quaternion desired_orientation_in_world
# This is the position of the control frame's origin expressed in endEffector.getBodyFixedFrame().
# By default, the control frame is coincident to endEffector.getBodyFixedFrame().
# The control frame is rigidly attached to the end-effector.
geometry_msgs/Point control_frame_position_in_end_effector
# This is the orientation of the control frame expressed in endEffector.getBodyFixedFrame().
# By default, the control frame is coincident to endEffector.getBodyFixedFrame().
geometry_msgs/Quaternion control_frame_orientation_in_end_effector
# The selection matrix is used to determinate which degree of freedom of the end-effector should
# be controlled.
# The selection frames coming along with the given selection matrix are used to determine to
# what reference frame the selected axes are referring to. For instance, if only the hand height
# in world should be controlled on the linear z component of the selection matrix should be
# selected and the reference frame should be world frame. When no reference frame is provided
# with the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed
# frame if not defined otherwise.
SelectionMatrix3DMessage angular_selection_matrix
# The selection matrix is used to determinate which degree of freedom of the end-effector should
# be controlled.
# The selection frames coming along with the given selection matrix are used to determine to
# what reference frame the selected axes are referring to. For instance, if only the hand height
# in world should be controlled on the linear z component of the selection matrix should be
# selected and the reference frame should be world frame. When no reference frame is provided
# with the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed
# frame if not defined otherwise.
SelectionMatrix3DMessage linear_selection_matrix
# Weight matrix used to define the priority of controlling the rotation around each axis on the solver side.
WeightMatrix3DMessage angular_weight_matrix
# Weight matrix used to define the priority of controlling the translation around each axis on the solver side.
WeightMatrix3DMessage linear_weight_matrix