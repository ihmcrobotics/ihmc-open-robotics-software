# General purpose message normally used to report the solution of a whole-body trajectory planner.
# Main usage is the IHMC WholeBodyTrajectoryToolbox.

int32 planning_result 0
float64[<=50] trajectory_times
KinematicsToolboxOutputStatus[<=50] robot_configurations