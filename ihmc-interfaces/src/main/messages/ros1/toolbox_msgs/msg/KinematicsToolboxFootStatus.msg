
# General purpose message normally used to report the solution of a whole-body inverse kinematics solver.
# Which hand to receive the object
# Field default value 255
int8 side

# Legged robot-specific contact information (false if not a legged robot)
bool foot_in_contact

# Relative position of the foot wrt the pelvis frame
geometry_msgs/Point relative_foot_position_from_pelvis

geometry_msgs/Quaternion relative_foot_orientation_from_pelvis

# Relative position of the foot wrt the pelvis frame at the beginning of a step
geometry_msgs/Point relative_foot_position_from_pelvis_step_start

geometry_msgs/Quaternion relative_foot_orientation_from_pelvis_step_start


