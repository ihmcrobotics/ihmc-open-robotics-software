# This message is part of the IHMC whole-body inverse kinematics module.

# When set to true, the solver will hold the current x and y coordinates of the center
# of mass. By 'current', it means that the solver will use the robot configuration data
# broadcasted by the controller to obtain the center of mass position.
bool hold_current_center_of_mass_xy_position true
# When set to true, the solver will hold the pose of the active support foot/feet.
bool hold_support_foot_positions true