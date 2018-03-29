# Message used to configure the IHMC whole-body trajetory planner.
# Main usage is the IHMC WholeBodyTrajectoryToolbox.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

int32 number_of_initial_guesses -1
int32 maximum_expansion_size -1
KinematicsToolboxOutputStatus initial_configuration