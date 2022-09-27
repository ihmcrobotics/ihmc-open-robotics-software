# Configure a constrained trajectory for a given end-effector.
# Main usage is the IHMC WholeBodyTrajectoryToolbox.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int32 end_effector_hash_code

float64[] waypoint_times

geometry_msgs/Pose[] waypoints

ihmc_common_msgs/SelectionMatrix3DMessage angular_selection_matrix

ihmc_common_msgs/SelectionMatrix3DMessage linear_selection_matrix

geometry_msgs/Point control_frame_position_in_end_effector

geometry_msgs/Quaternion control_frame_orientation_in_end_effector

# Field default value -1.0
float64 weight


