# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

bool goals_valid

controller_msgs/FootstepDataMessage start

controller_msgs/FootstepDataMessage[] original_goals

controller_msgs/FootstepDataMessage[] path_plan

bool[] footstep_unknown

float64 sub_optimality

# Field default value 1000000.0
float64 path_cost


