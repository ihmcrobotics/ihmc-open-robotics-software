# General purpose message normally used to report the solution of a whole-body trajectory planner.
# Main usage is the IHMC WholeBodyTrajectoryToolbox.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

# 0: not completed.
# 1: fail to find initial guess.
# 2: fail to complete expanding tree.
# 3: fail to optimize path.
# 4: solution is available.
int32 planning_result 0
float64[<=50] trajectory_times
KinematicsToolboxOutputStatus[<=50] robot_configurations