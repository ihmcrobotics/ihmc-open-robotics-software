
# Parent state fields
behavior_msgs/ActionNodeStateMessage state

# Definition
behavior_msgs/FootstepPlanActionDefinitionMessage definition

# The footsteps, with a maximum of 50
behavior_msgs/FootstepPlanActionFootstepStateMessage[] footsteps

# Total number of footsteps; used for walking actions
uint16 total_number_of_footsteps

# Incomplete footsteps; used for walking actions
uint16 number_of_incomplete_footsteps


