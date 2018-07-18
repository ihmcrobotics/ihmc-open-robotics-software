# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When set to true, the solver will hold the current x and y coordinates of the center
# of mass. By 'current', it means that the solver will use the robot configuration data
# broadcasted by the controller to obtain the center of mass position.
# Field default value True
bool hold_current_center_of_mass_xy_position

# When set to true, the solver will hold the pose of the active support foot/feet.
# Field default value True
bool hold_support_foot_positions


