# Configure a constrained trajectory for a given end-effector.
# Main usage is the IHMC WholeBodyTrajectoryToolbox.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

int64 end_effector_name_based_hash_code
float64[] waypoint_times
geometry_msgs/Pose[] waypoints
SelectionMatrix3DMessage angular_selection_matrix
SelectionMatrix3DMessage linear_selection_matrix

geometry_msgs/Point control_frame_position_in_end_effector
geometry_msgs/Quaternion control_frame_orientation_in_end_effector

float64 weight