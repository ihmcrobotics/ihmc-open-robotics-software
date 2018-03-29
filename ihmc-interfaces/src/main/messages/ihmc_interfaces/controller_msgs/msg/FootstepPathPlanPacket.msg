# (Obsolete) This message is part of the old IHMC footstep planning module.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

bool goals_valid
FootstepDataMessage start
FootstepDataMessage[] original_goals
FootstepDataMessage[] path_plan
bool[] footstep_unknown
float64 sub_optimality
float64 path_cost inf