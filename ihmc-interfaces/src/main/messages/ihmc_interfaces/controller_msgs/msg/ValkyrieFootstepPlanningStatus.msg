# ID that matches the plan_id of the active request packet
int32 plan_id -1

# Planner is actively planning. If any other status is indicated, the planner has terminated
byte STATUS_PLANNING = 0
# A solution was found that reaches the goal
byte STATUS_FOUND_SOLUTION = 1
# The timeout was reached and the planner did not find valid solution
byte STATUS_TIMED_OUT = 2
# Planner exhaustively searched all possibilities and could not find a plan
byte STATUS_NO_SOLUTION_EXISTS = 3
# The goal steps in the request packet are not valid
byte STATUS_INVALID_GOAL = 4

# Current planner status
byte planner_status 255

# Contains the current best solution, i.e. lowest cost + heuristic
# If the planner found a solution, this contains the solution path
FootstepDataListMessage footstep_data_list