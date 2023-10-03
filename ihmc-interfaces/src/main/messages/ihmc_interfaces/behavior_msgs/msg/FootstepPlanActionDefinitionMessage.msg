# Used for syncing action sequences
behavior_msgs/ActionInformationMessage action_information

# Name of the parent frame the footsteps are expressed in
string[<=1000] parent_frame

# The footsteps, with a maximum of 50
behavior_msgs/FootstepActionDefinitionMessage[<=50] footsteps

# Swing duration
float64 swing_duration

# Transfer duration
float64 transfer_duration