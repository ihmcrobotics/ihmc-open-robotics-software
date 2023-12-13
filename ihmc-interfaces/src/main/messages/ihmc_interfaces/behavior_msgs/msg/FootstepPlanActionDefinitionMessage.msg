# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Name of the parent frame the footsteps are expressed in
string parent_frame_name

# Swing duration
float64 swing_duration

# Transfer duration
float64 transfer_duration

# The footsteps, with a maximum of 50
behavior_msgs/FootstepPlanActionFootstepDefinitionMessage[<=50] footsteps