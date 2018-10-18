uint32 sequence_id

int64 end_effector_name_based_hash_code

float64[] key_frame_times

geometry_msgs/Pose[] key_frame_poses

# The selection frames coming along with the given selection matrix are used to determine to
# what reference frame the selected axes are referring to. For instance, if only the hand height
# in world should be controlled on the linear z component of the selection matrix should be
# selected and the reference frame should be world frame. When no reference frame is provided
# with the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed
# frame if not defined otherwise.
controller_msgs/SelectionMatrix3DMessage angular_selection_matrix

# The selection matrix is used to determinate which degree of freedom of the end-effector should
# be controlled.
# The selection frames coming along with the given selection matrix are used to determine to
# what reference frame the selected axes are referring to. For instance, if only the hand height
# in world should be controlled on the linear z component of the selection matrix should be
# selected and the reference frame should be world frame. When no reference frame is provided
# with the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed
# frame if not defined otherwise.
controller_msgs/SelectionMatrix3DMessage linear_selection_matrix

# Weight matrix used to define the priority of controlling the rotation around each axis on the solver side.
controller_msgs/WeightMatrix3DMessage angular_weight_matrix

# Weight matrix used to define the priority of controlling the translation around each axis on the solver side.
controller_msgs/WeightMatrix3DMessage linear_weight_matrix

geometry_msgs/Point control_frame_position_in_end_effector

geometry_msgs/Quaternion control_frame_orientation_in_end_effector

float64[] allowable_position_displacement

float64[] allowable_orientation_displacement


