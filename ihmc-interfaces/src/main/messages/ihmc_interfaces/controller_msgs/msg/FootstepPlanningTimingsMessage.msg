# This message is part of the IHMC footstep planning module.

# Total time measured in the planner process between receiving request message and publishing output message
float64 total_elapsed_seconds

# Elapsed time between receiving request message and starting to plan body path
float64 time_before_planning_seconds

# Elapsed time for planning body path
float64 time_planning_body_path_seconds

# Elapsed time for step planning
float64 time_planning_steps_seconds

# Number of iterations performed during step planning
int64 step_planning_iterations