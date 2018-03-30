# (Obsolete) This message is part of the old IHMC footstep planning module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

bool goals_valid
FootstepDataMessage start
FootstepDataMessage[] original_goals
FootstepDataMessage[] path_plan
bool[] footstep_unknown
float64 sub_optimality
float64 path_cost inf