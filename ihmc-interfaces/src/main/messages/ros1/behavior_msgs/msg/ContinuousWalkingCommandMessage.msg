
# flag to enable/disable publishing to controller
bool publish_to_controller

# flag to enable/disable continuous walking state machine
bool enable_continuous_walking

# number of steps to send to controller
uint16 number_of_steps_to_send

# forward joystick value
float64 forward_value

# lateral joystick value
float64 lateral_value

# turning joystick value
float64 turning_value

# flag to enable/disable hybrid planning
bool use_hybrid_planner

# flag to enable/disable planning with astar planner
bool use_astar_footstep_planner

# flag to enable/disable planning with monte-carlo footstep planner
bool use_monte_carlo_footstep_planner

# flag to enable/disable using previous plan as reference
bool use_previous_plan_as_reference

# flag to enable/disable using monte-carlo plan as reference
bool use_monte_carlo_plan_as_reference


