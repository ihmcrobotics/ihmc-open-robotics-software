# General purpose message normally used to report the solution of a whole-body trajectory planner.
# Main usage is the IHMC WholeBodyTrajectoryToolbox.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# 0: not completed.
# 1: fail to find initial guess.
# 2: fail to complete expanding tree.
# 3: fail to optimize path.
# 4: solution is available.
int32 planning_result

float64[] trajectory_times

toolbox_msgs/KinematicsToolboxOutputStatus[] robot_configurations


