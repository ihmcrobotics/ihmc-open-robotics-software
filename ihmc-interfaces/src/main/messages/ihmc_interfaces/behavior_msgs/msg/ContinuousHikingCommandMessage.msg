# This message sends the current request for the Continuous Hiking process.
# This holds parameters on how the Continuous Hiking process will behave.

# Flag to start Continuous Hiking
bool enable_continuous_hiking

# Flag that allows the user to set a number of steps that will be taken, and Continuous Hiking will stop after that. This way the robot doesn't walk forever
uint32 steps_before_safety_stop

# Flag to walk straight forward with Continuous Hiking. This is generally a keyboard input, and is different then trying to walk with the Joystick
bool walk_forwards

# Flag to square up to the goal when finishing walking. We may want the robot to end at an exact location.
# Setting this to true tells the robot to step on the final goal positions.
bool square_up_to_goal

# Flag to enable/disable planning with A* planner. This is the default planner to use
bool use_astar_footstep_planner

# Flag to enable/disable planning with monte-carlo footstep planner
bool use_monte_carlo_footstep_planner

# Flag to enable/disable using previous plan as reference. Generally this should be set to true to reduce planning time
bool use_previous_plan_as_reference

# Flag to enable/disable using monte-carlo plan as reference
bool use_monte_carlo_plan_as_reference

# Walk with the Joystick Controller
bool use_joystick_controller

# Forward joystick value
float64 forward_value

# Backward walking from Joystick Controller
bool walk_backwards

# Lateral joystick value
float64 lateral_value

# Turning joystick value
float64 turning_value