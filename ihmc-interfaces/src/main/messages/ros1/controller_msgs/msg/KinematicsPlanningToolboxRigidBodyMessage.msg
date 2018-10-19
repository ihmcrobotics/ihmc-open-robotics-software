uint32 sequence_id

int64 end_effector_name_based_hash_code

# This is the list of desired times for each key frames.
float64[] key_frame_times

# This is the list of desired key frames for end effector.
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

# This is the position of the control frame's origin expressed in endEffector.getBodyFixedFrame().
# By default, the control frame is coincident to endEffector.getBodyFixedFrame().
# The control frame is rigidly attached to the end-effector.
geometry_msgs/Point control_frame_position_in_end_effector

# This is the orientation of the control frame expressed in endEffector.getBodyFixedFrame().
# By default, the control frame is coincident to endEffector.getBodyFixedFrame().
geometry_msgs/Quaternion control_frame_orientation_in_end_effector

# This is the list of allowable displacement for position when the trajectory is modified.
# The unit is meter and will be applied samely on x-y-z.
float64[] allowable_position_displacement

# This is the list of allowable displacement for orientation when the trajectory is modified.
# The unit is radian.
float64[] allowable_orientation_displacement


