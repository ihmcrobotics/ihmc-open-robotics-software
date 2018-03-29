# General purpose message normally used to report the solution of a whole-body inverse kinematics solver.
# Main usage is for the IHMC KinematicsToolbox.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

int32 joint_name_hash
float32[] desired_joint_angles

geometry_msgs/Vector3 desired_root_translation
geometry_msgs/Quaternion desired_root_orientation

float64 solution_quality