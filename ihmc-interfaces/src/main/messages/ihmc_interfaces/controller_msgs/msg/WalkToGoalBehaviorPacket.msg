# This message is part of the IHMC humanoid behavior module.

byte WALK_TO_GOAL_ACTION_FIND_PATH = 0
byte WALK_TO_GOAL_ACTION_EXECUTE = 1
byte WALK_TO_GOAL_ACTION_EXECUTE_UNKNOWN = 2
byte WALK_TO_GOAL_ACTION_STOP = 3

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

byte walk_to_goal_action -1
float64 x_goal
float64 y_goal
float64 theta_goal

byte goal_robot_side -1