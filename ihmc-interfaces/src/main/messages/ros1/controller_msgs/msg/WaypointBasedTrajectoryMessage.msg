# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int64 end_effector_name_based_hash_code

float64[] waypoint_times

geometry_msgs/Pose[] waypoints

controller_msgs/SelectionMatrix3DMessage angular_selection_matrix

controller_msgs/SelectionMatrix3DMessage linear_selection_matrix

geometry_msgs/Point control_frame_position_in_end_effector

geometry_msgs/Quaternion control_frame_orientation_in_end_effector

# Field default value -1.0
float64 weight


