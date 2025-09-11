
# General purpose message normally used to report the solution of a whole-body inverse kinematics solver.
# Which hand to receive the object
# Field default value 255
int8 side

# Legged robot-specific contact information (false if not a legged robot)
bool foot_in_contact

geometry_msgs/Point desired_relative_foot_position_from_stance

geometry_msgs/Quaternion desired_relative_foot_orientation_from_stance


