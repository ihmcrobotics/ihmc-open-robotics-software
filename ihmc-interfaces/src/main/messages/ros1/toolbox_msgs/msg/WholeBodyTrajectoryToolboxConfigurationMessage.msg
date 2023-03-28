# Message used to configure the IHMC whole-body trajetory planner.
# Main usage is the IHMC WholeBodyTrajectoryToolbox.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value -1
int32 number_of_initial_guesses

# Field default value -1
int32 maximum_expansion_size

toolbox_msgs/KinematicsToolboxOutputStatus initial_configuration


