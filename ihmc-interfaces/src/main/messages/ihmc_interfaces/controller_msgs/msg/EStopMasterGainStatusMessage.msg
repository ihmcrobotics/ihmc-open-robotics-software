# A message for allowing operator to observe the status of various startup/shutdown variables often required for robot operation

# Current SOFT-E-STOP value
bool is_estopped

# Is robot faulted
bool robot_is_faulted

# Is robot servod
bool robot_is_servod

# Is robot calibrated
bool robot_is_calibrated

# Is publishing ROS commands to robot enabled
bool publishing_to_robot_is_enabled

# Are the actuators enabled
bool actuators_are_enabled

# The current master gain
float64 current_master_gain
