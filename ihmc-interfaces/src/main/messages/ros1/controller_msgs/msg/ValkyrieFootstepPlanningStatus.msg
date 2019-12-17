
# Planner is actively planning. If any other status is indicated, the planner has terminated
int8 STATUS_PLANNING=0

# A solution was found that reaches the goal
int8 STATUS_FOUND_SOLUTION=1

# The timeout was reached and the planner did not find valid solution
int8 STATUS_TIMED_OUT=2

# Planner exhaustively searched all possibilities and could not find a plan
int8 STATUS_NO_SOLUTION_EXISTS=3

# The goal steps in the request packet are not valid
int8 STATUS_INVALID_GOAL=4

# ID that matches the plan_id of the active request packet
# Field default value -1
int32 plan_id

# Current planner status
# Field default value 255
int8 planner_status

# Contains the current best solution, i.e. lowest cost + heuristic
# If the planner found a solution, this contains the solution path
controller_msgs/FootstepDataListMessage footstep_data_list


